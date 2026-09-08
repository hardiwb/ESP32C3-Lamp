#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_sntp.h"
#include "lwip/apps/sntp.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "ds1302.h"
#include "lamp_config.h"

static const char *TAG = "desk_lamp";
static EventGroupHandle_t wifi_events;
static SemaphoreHandle_t state_lock;
static volatile int wifi_retry_count;
static uint16_t warm_level;
static uint16_t cool_level;
static uint16_t master_brightness = 700;
static char lamp_mode[5] = "off";
static int64_t timer_deadline_us;
static bool timer_turn_on;
static char resume_mode[5] = "warm";
static bool restore_on_boot;
static const char *clock_source = "none";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAX_RETRY 10
#define LAMP_MAX_DUTY ((1U << LAMP_PWM_RESOLUTION_BITS) - 1U)
#define LAMP_FADE_TIME_MS 1000
#define SCHEDULE_COUNT 3

typedef struct {
    bool enabled;
    uint16_t on_minute;
    uint16_t off_minute;
} daily_schedule_t;

static daily_schedule_t schedules[SCHEDULE_COUNT] = {
    {.enabled = false, .on_minute = 7 * 60, .off_minute = 22 * 60},
    {.enabled = false, .on_minute = 7 * 60, .off_minute = 22 * 60},
    {.enabled = false, .on_minute = 7 * 60, .off_minute = 22 * 60},
};

enum {
    PWM_MODE_WARM = 0,
    PWM_MODE_COOL = 1,
};

static void apply_output(void)
{
    uint16_t warm;
    uint16_t cool;

    xSemaphoreTake(state_lock, portMAX_DELAY);
    warm = warm_level;
    cool = cool_level;
    xSemaphoreGive(state_lock);

    ESP_ERROR_CHECK(ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                                 warm, LAMP_FADE_TIME_MS, LEDC_FADE_NO_WAIT));
    ESP_ERROR_CHECK(ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                                                 cool, LAMP_FADE_TIME_MS, LEDC_FADE_NO_WAIT));
}

static void set_mode(const char *mode)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    if (strcmp(mode, "warm") == 0) {
        strcpy(lamp_mode, "warm");
        strcpy(resume_mode, "warm");
        warm_level = master_brightness;
        cool_level = 0;
    } else if (strcmp(mode, "cool") == 0) {
        strcpy(lamp_mode, "cool");
        strcpy(resume_mode, "cool");
        warm_level = 0;
        cool_level = master_brightness;
    } else if (strcmp(mode, "mix") == 0) {
        strcpy(lamp_mode, "mix");
        strcpy(resume_mode, "mix");
        warm_level = master_brightness;
        cool_level = master_brightness;
    } else {
        strcpy(lamp_mode, "off");
        warm_level = 0;
        cool_level = 0;
    }
    xSemaphoreGive(state_lock);
    apply_output();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGI(TAG, "retrying Wi-Fi connection");
        } else {
            xEventGroupSetBits(wifi_events, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        wifi_retry_count = 0;
        ESP_LOGI(TAG, "connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = LAMP_WIFI_SSID,
            .password = LAMP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_events,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi ready");
    } else {
        ESP_LOGW(TAG, "Wi-Fi connection failed; reboot or check credentials");
    }
}

static void pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LAMP_PWM_RESOLUTION_BITS,
        .freq_hz = LAMP_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t warm_channel = {
        .gpio_num = LAMP_WARM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t cool_channel = warm_channel;
    cool_channel.gpio_num = LAMP_COOL_GPIO;
    cool_channel.channel = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&warm_channel));
    ESP_ERROR_CHECK(ledc_channel_config(&cool_channel));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Desk Lamp</title><style>*{box-sizing:border-box}body{font-family:system-ui,sans-serif;max-width:1000px;margin:20px auto;padding:0 16px;background:#f4efe6;color:#25231f}"
    "main{background:#fffaf2;padding:4px 24px;border:1px solid #ded5c6;border-radius:12px;box-shadow:0 8px 24px #6b59421c}.column{min-width:0}.panel{min-width:0;padding:22px 0}.panel+.panel{border-top:1px solid #ded5c6}"
    "h1,h2{margin:0 0 14px;line-height:1.15}h1{font-size:2rem}h2{font-size:1.55rem}p{margin:10px 0 16px}"
    "button{font:inherit;border:1px solid #a87945;background:#fff;padding:10px 14px;border-radius:8px;cursor:pointer}button.active{background:#a87945;color:white}"
    "input,select{font:inherit;width:100%;padding:8px;accent-color:#a87945}input[type=range]{padding:0}.mode-buttons,.actions{display:flex;flex-wrap:wrap;gap:8px;margin:14px 0}"
    ".row{display:grid;grid-template-columns:1fr 1fr;gap:12px}.inline{display:flex;gap:8px;align-items:center;margin-bottom:10px}.inline input{width:auto}.schedule-slot{padding:12px 0}.schedule-slot+.schedule-slot{border-top:1px solid #e7dfd3}.schedule-slot .inline{font-weight:600}"
    "#brightness{margin-top:4px}#firmware input{margin:6px 0 14px}#ota-status,#status,#timer-status,#clock{color:#6f665d}#clock{margin-top:0}"
    "@media(min-width:800px){main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));column-gap:32px;align-items:start}}"
    "@media(max-width:520px){.row{grid-template-columns:1fr}body{margin-top:8px}main{padding:4px 16px}}</style></head>"
    "<body><main><div class=column><section class=panel id=lamp-control><h1>Desk Lamp</h1><p>Choose the light temperature.</p>"
    "<div class=mode-buttons><button data-mode=off>Off</button><button data-mode=warm>Warm</button><button data-mode=mix>Balanced</button><button data-mode=cool>Cool</button></div>"
    "<label for=brightness>Brightness</label><input id=brightness type=range min=0 max=100 step=1 value=70><p id=status>Loading...</p></section>"
    "<section class=panel id=timer><h2>Timer</h2><div class=row><label>Minutes<input id=timer-minutes type=number min=1 max=1440 value=30></label>"
    "<label>Action<select id=timer-action><option value=off>Turn off</option><option value=on>Turn on</option></select></label></div>"
    "<div class=actions><button id=timer-set>Start timer</button><button id=timer-cancel>Cancel</button></div><p id=timer-status>No active timer</p></section>"
    "<section class=panel id=firmware><h2>Firmware update</h2><p>Select a firmware .bin built for this device.</p>"
    "<input id=ota-file type=file accept='.bin,application/octet-stream'><div class=actions><button id=ota-button>Install update</button></div><p id=ota-status></p></section></div>"
    "<div class=column><section class=panel id=schedule><h2>Daily schedules</h2><p id=clock>Synchronizing...</p>"
    "<div class=schedule-slot><label class=inline><input id=schedule-1-enabled type=checkbox> Schedule 1</label><div class=row><label>Turn on<input id=schedule-1-on type=time value=07:00></label><label>Turn off<input id=schedule-1-off type=time value=22:00></label></div></div>"
    "<div class=schedule-slot><label class=inline><input id=schedule-2-enabled type=checkbox> Schedule 2</label><div class=row><label>Turn on<input id=schedule-2-on type=time value=07:00></label><label>Turn off<input id=schedule-2-off type=time value=22:00></label></div></div>"
    "<div class=schedule-slot><label class=inline><input id=schedule-3-enabled type=checkbox> Schedule 3</label><div class=row><label>Turn on<input id=schedule-3-on type=time value=07:00></label><label>Turn off<input id=schedule-3-off type=time value=22:00></label></div></div>"
    "<div class=actions><button id=schedule-save>Save schedule</button><button id=deep-sleep>Sleep until next schedule</button></div>"
    "<p>Deep sleep turns everything off and wakes at the next enabled start time. Replug power to wake it early.</p></section></div>"
    "</main><script>const status=document.querySelector('#status'),slider=document.querySelector('#brightness'),clock=document.querySelector('#clock'),timerStatus=document.querySelector('#timer-status');"
    "async function refresh(){let r=await fetch('/api/state'),s=await r.json();let percent=Math.round(s.brightness/10.23);slider.value=percent;status.textContent=s.mode+' / '+percent+'%';document.querySelectorAll('button[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode===s.mode))}"
    "async function update(data){await fetch('/api/state',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});refresh()}"
    "document.querySelectorAll('button[data-mode]').forEach(b=>b.onclick=()=>update({mode:b.dataset.mode}));"
    "slider.oninput=()=>status.textContent='Brightness '+slider.value+'%';slider.onchange=()=>update({brightness:Math.round(Number(slider.value)*10.23)});"
    "let scheduleDirty=false;async function automation(data){await fetch('/api/automation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});await refreshAutomation(true)}"
    "async function refreshAutomation(force=false){try{let r=await fetch('/api/automation'),a=await r.json();clock.textContent=a.synced?a.local_time+' ('+a.timezone+', '+(a.source==='rtc'?'RTC':'internet')+')':'Waiting for RTC or internet time...';"
    "if(force||!scheduleDirty)a.schedules.forEach((s,i)=>{let n=i+1;document.querySelector('#schedule-'+n+'-enabled').checked=s.enabled;document.querySelector('#schedule-'+n+'-on').value=s.on;document.querySelector('#schedule-'+n+'-off').value=s.off});"
    "if(a.timer.active){let m=Math.floor(a.timer.remaining_seconds/60),s=a.timer.remaining_seconds%60;timerStatus.textContent='Will turn '+a.timer.action+' in '+m+':'+String(s).padStart(2,'0')}else timerStatus.textContent='No active timer'}catch(e){clock.textContent='Clock unavailable'}}"
    "document.querySelector('#timer-set').onclick=()=>automation({timer_minutes:Number(document.querySelector('#timer-minutes').value),timer_action:document.querySelector('#timer-action').value});"
    "document.querySelector('#timer-cancel').onclick=()=>automation({cancel_timer:true});document.querySelectorAll('.schedule-slot input').forEach(e=>e.onchange=()=>scheduleDirty=true);"
    "document.querySelector('#schedule-save').onclick=async()=>{let d={};for(let n=1;n<=3;n++){d['schedule_'+n+'_enabled']=document.querySelector('#schedule-'+n+'-enabled').checked;d['schedule_'+n+'_on']=document.querySelector('#schedule-'+n+'-on').value;d['schedule_'+n+'_off']=document.querySelector('#schedule-'+n+'-off').value}await automation(d);scheduleDirty=false};"
    "document.querySelector('#deep-sleep').onclick=async function(){if(scheduleDirty){clock.textContent='Save the schedule before sleeping.';return}if(!confirm('Sleep until the next enabled schedule starts? You can replug power to wake early.'))return;this.disabled=true;try{let r=await fetch('/api/sleep',{method:'POST'});if(!r.ok)throw Error(await r.text());let s=await r.json(),m=Math.floor(s.wake_seconds/60);clock.textContent='Sleeping until the next schedule (in about '+m+' minute'+(m===1?'':'s')+').'}catch(e){clock.textContent='Could not enter deep sleep: '+e.message;this.disabled=false}};"
    "document.querySelector('#ota-button').onclick=()=>{let f=document.querySelector('#ota-file').files[0],s=document.querySelector('#ota-status'),b=document.querySelector('#ota-button');"
    "if(!f){s.textContent='Choose a firmware file first.';return}if(!confirm('Install '+f.name+' and restart the lamp?'))return;"
    "b.disabled=true;let x=new XMLHttpRequest;x.open('POST','/api/ota');x.setRequestHeader('Content-Type','application/octet-stream');"
    "x.upload.onprogress=e=>{if(e.lengthComputable)s.textContent='Uploading '+Math.round(e.loaded/e.total*100)+'%'};"
    "x.onload=()=>{s.textContent=x.status===200?'Update installed. Lamp is restarting...':'Update failed: '+x.responseText;if(x.status!==200)b.disabled=false};"
    "x.onerror=()=>{s.textContent='Upload connection failed.';b.disabled=false};x.send(f)};refresh();refreshAutomation();setInterval(refreshAutomation,1000);</script></body></html>";

static esp_err_t index_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    return httpd_resp_send(request, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static const char *current_mode(void)
{
    return lamp_mode;
}

static esp_err_t state_get_handler(httpd_req_t *request)
{
    char response[128];
    xSemaphoreTake(state_lock, portMAX_DELAY);
    snprintf(response, sizeof(response), "{\"mode\":\"%s\",\"brightness\":%u}", current_mode(), master_brightness);
    xSemaphoreGive(state_lock);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

static void state_load(void)
{
    nvs_handle_t handle;
    if (nvs_open("lamp", NVS_READONLY, &handle) != ESP_OK) return;

    uint16_t saved_brightness;
    if (nvs_get_u16(handle, "brightness", &saved_brightness) == ESP_OK &&
        saved_brightness <= LAMP_MAX_DUTY) {
        master_brightness = saved_brightness;
    }

    char saved_mode[sizeof(resume_mode)];
    size_t mode_length = sizeof(saved_mode);
    if (nvs_get_str(handle, "last_mode", saved_mode, &mode_length) == ESP_OK &&
        (strcmp(saved_mode, "warm") == 0 || strcmp(saved_mode, "cool") == 0 ||
         strcmp(saved_mode, "mix") == 0)) {
        strcpy(resume_mode, saved_mode);
    }
    uint8_t saved_power = 0;
    if (nvs_get_u8(handle, "power_on", &saved_power) == ESP_OK) {
        restore_on_boot = saved_power != 0;
    }
    nvs_close(handle);
}

static esp_err_t state_save(void)
{
    uint16_t brightness;
    char mode[sizeof(resume_mode)];
    bool power_on;
    xSemaphoreTake(state_lock, portMAX_DELAY);
    brightness = master_brightness;
    strcpy(mode, resume_mode);
    power_on = strcmp(lamp_mode, "off") != 0;
    xSemaphoreGive(state_lock);

    nvs_handle_t handle;
    esp_err_t result = nvs_open("lamp", NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;
    result = nvs_set_u16(handle, "brightness", brightness);
    if (result == ESP_OK) result = nvs_set_str(handle, "last_mode", mode);
    if (result == ESP_OK) result = nvs_set_u8(handle, "power_on", power_on ? 1 : 0);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

static esp_err_t state_post_handler(httpd_req_t *request)
{
    char body[256];
    int received = httpd_req_recv(request, body, sizeof(body) - 1);
    if (received <= 0) return ESP_FAIL;
    body[received] = '\0';

    char mode[12];
    int brightness;
    bool mode_changed = false;
    bool brightness_changed = false;
    char *mode_key = strstr(body, "\"mode\"");
    char *brightness_key = strstr(body, "\"brightness\"");
    if (brightness_key != NULL && sscanf(brightness_key + 12, "%*[^0-9]%d", &brightness) == 1) {
        if (brightness < 0) brightness = 0;
        if (brightness > LAMP_MAX_DUTY) brightness = LAMP_MAX_DUTY;
        xSemaphoreTake(state_lock, portMAX_DELAY);
        master_brightness = (uint16_t)brightness;
        if (strcmp(lamp_mode, "warm") == 0) {
            warm_level = master_brightness;
        } else if (strcmp(lamp_mode, "cool") == 0) {
            cool_level = master_brightness;
        } else if (strcmp(lamp_mode, "mix") == 0) {
            warm_level = master_brightness;
            cool_level = master_brightness;
        }
        xSemaphoreGive(state_lock);
        brightness_changed = true;
    }
    if (mode_key != NULL) {
        char *mode_value = strchr(mode_key + 6, ':');
        if (mode_value != NULL && sscanf(mode_value + 1, " \"%11[^\"]\"", mode) == 1) {
            set_mode(mode);
            mode_changed = true;
        }
    }
    if (brightness_changed && !mode_changed) {
        apply_output();
    }
    if ((brightness_changed || mode_changed) && state_save() != ESP_OK) {
        httpd_resp_set_status(request, "500 Internal Server Error");
        return httpd_resp_sendstr(request, "Could not save lamp state");
    }
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, "{\"ok\":true}");
}

static bool clock_is_synced(void)
{
    time_t now;
    time(&now);
    return now > 1609459200; /* 2021-01-01 */
}

static void clock_start(void)
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, LAMP_NTP_SERVER);
    sntp_init();
    ESP_LOGI(TAG, "internet clock started using %s (%s)", LAMP_NTP_SERVER, LAMP_TIMEZONE);
}

static void rtc_start(void)
{
    setenv("TZ", LAMP_TIMEZONE, 1);
    tzset();
    ds1302_init((gpio_num_t)LAMP_RTC_CLK_GPIO, (gpio_num_t)LAMP_RTC_DATA_GPIO,
                (gpio_num_t)LAMP_RTC_RST_GPIO);

    struct tm rtc_time;
    if (!ds1302_read_time(&rtc_time)) {
        ESP_LOGW(TAG, "DS1302 time is not set; waiting for internet clock");
        return;
    }
    time_t epoch = mktime(&rtc_time);
    if (epoch <= 1609459200) {
        ESP_LOGW(TAG, "DS1302 returned an invalid date; waiting for internet clock");
        return;
    }
    struct timeval system_time = {.tv_sec = epoch, .tv_usec = 0};
    settimeofday(&system_time, NULL);
    clock_source = "rtc";
    ESP_LOGI(TAG, "system clock restored from DS1302");
}

static void schedule_load(void)
{
    static const char *enabled_keys[SCHEDULE_COUNT] = {"sched_en", "sched_en2", "sched_en3"};
    static const char *on_keys[SCHEDULE_COUNT] = {"sched_on", "sched_on2", "sched_on3"};
    static const char *off_keys[SCHEDULE_COUNT] = {"sched_off", "sched_off2", "sched_off3"};
    nvs_handle_t handle;
    if (nvs_open("lamp", NVS_READONLY, &handle) != ESP_OK) return;
    for (size_t i = 0; i < SCHEDULE_COUNT; i++) {
        uint8_t enabled = 0;
        nvs_get_u8(handle, enabled_keys[i], &enabled);
        nvs_get_u16(handle, on_keys[i], &schedules[i].on_minute);
        nvs_get_u16(handle, off_keys[i], &schedules[i].off_minute);
        if (schedules[i].on_minute >= 24 * 60) schedules[i].on_minute = 7 * 60;
        if (schedules[i].off_minute >= 24 * 60) schedules[i].off_minute = 22 * 60;
        schedules[i].enabled = enabled != 0;
    }
    nvs_close(handle);
}

static esp_err_t schedule_save(void)
{
    static const char *enabled_keys[SCHEDULE_COUNT] = {"sched_en", "sched_en2", "sched_en3"};
    static const char *on_keys[SCHEDULE_COUNT] = {"sched_on", "sched_on2", "sched_on3"};
    static const char *off_keys[SCHEDULE_COUNT] = {"sched_off", "sched_off2", "sched_off3"};
    nvs_handle_t handle;
    esp_err_t result = nvs_open("lamp", NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;
    for (size_t i = 0; i < SCHEDULE_COUNT && result == ESP_OK; i++) {
        result = nvs_set_u8(handle, enabled_keys[i], schedules[i].enabled ? 1 : 0);
        if (result == ESP_OK) result = nvs_set_u16(handle, on_keys[i], schedules[i].on_minute);
        if (result == ESP_OK) result = nvs_set_u16(handle, off_keys[i], schedules[i].off_minute);
    }
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

static void turn_on(void)
{
    char mode[sizeof(resume_mode)];
    xSemaphoreTake(state_lock, portMAX_DELAY);
    strcpy(mode, resume_mode);
    xSemaphoreGive(state_lock);
    set_mode(mode);
}

static void restore_state_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(3000));
    turn_on();
    ESP_LOGI(TAG, "restored saved lamp state after startup delay");
    vTaskDelete(NULL);
}

static void automation_task(void *arg)
{
    (void)arg;
    int64_t last_schedule_minute = -1;
    bool rtc_updated_from_ntp = false;
    for (;;) {
        int64_t deadline;
        bool turn_on_at_deadline;
        xSemaphoreTake(state_lock, portMAX_DELAY);
        deadline = timer_deadline_us;
        turn_on_at_deadline = timer_turn_on;
        xSemaphoreGive(state_lock);

        if (deadline > 0 && esp_timer_get_time() >= deadline) {
            xSemaphoreTake(state_lock, portMAX_DELAY);
            if (timer_deadline_us == deadline) timer_deadline_us = 0;
            xSemaphoreGive(state_lock);
            if (turn_on_at_deadline) turn_on(); else set_mode("off");
        }

        time_t now;
        time(&now);
        if (!rtc_updated_from_ntp && sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
            struct tm local;
            localtime_r(&now, &local);
            if (ds1302_write_time(&local)) {
                clock_source = "internet";
                ESP_LOGI(TAG, "DS1302 updated from internet time");
            } else {
                ESP_LOGW(TAG, "internet time synchronized, but DS1302 could not be verified");
                clock_source = "internet";
            }
            rtc_updated_from_ntp = true;
        }
        int64_t epoch_minute = (int64_t)now / 60;
        if (clock_is_synced() && epoch_minute != last_schedule_minute) {
            struct tm local;
            localtime_r(&now, &local);
            uint16_t minute = (uint16_t)(local.tm_hour * 60 + local.tm_min);
            daily_schedule_t current[SCHEDULE_COUNT];
            xSemaphoreTake(state_lock, portMAX_DELAY);
            memcpy(current, schedules, sizeof(current));
            xSemaphoreGive(state_lock);

            bool any_enabled = false;
            bool any_active = false;
            bool turn_on_now = false;
            bool turn_off_now = false;
            for (size_t i = 0; i < SCHEDULE_COUNT; i++) {
                daily_schedule_t *schedule = &current[i];
                if (!schedule->enabled || schedule->on_minute == schedule->off_minute) continue;
                any_enabled = true;
                bool active = schedule->on_minute < schedule->off_minute
                    ? minute >= schedule->on_minute && minute < schedule->off_minute
                    : minute >= schedule->on_minute || minute < schedule->off_minute;
                any_active = any_active || active;
                turn_on_now = turn_on_now || minute == schedule->on_minute;
                turn_off_now = turn_off_now || minute == schedule->off_minute;
            }
            if (last_schedule_minute < 0 && any_enabled) {
                if (any_active) turn_on(); else set_mode("off");
            } else if (turn_on_now) {
                turn_on();
            } else if (turn_off_now && !any_active) {
                set_mode("off");
            }
            last_schedule_minute = epoch_minute;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static bool json_bool(const char *body, const char *key, bool *value)
{
    char *field = strstr(body, key);
    if (field == NULL) return false;
    char *separator = strchr(field, ':');
    if (separator == NULL) return false;
    separator++;
    while (*separator == ' ') separator++;
    if (strncmp(separator, "true", 4) == 0) *value = true;
    else if (strncmp(separator, "false", 5) == 0) *value = false;
    else return false;
    return true;
}

static bool json_time(const char *body, const char *key, uint16_t *minute)
{
    char *field = strstr(body, key);
    if (field == NULL) return false;
    char *separator = strchr(field, ':');
    int hour;
    int min;
    if (separator == NULL || sscanf(separator + 1, " \"%2d:%2d\"", &hour, &min) != 2) return false;
    if (hour < 0 || hour > 23 || min < 0 || min > 59) return false;
    *minute = (uint16_t)(hour * 60 + min);
    return true;
}

static esp_err_t automation_get_handler(httpd_req_t *request)
{
    char response[640];
    char local_time[40] = "Not synchronized";
    bool synced = clock_is_synced();
    if (synced) {
        time_t now;
        struct tm local;
        time(&now);
        localtime_r(&now, &local);
        strftime(local_time, sizeof(local_time), "%Y-%m-%d %H:%M:%S", &local);
    }

    int64_t now_us = esp_timer_get_time();
    xSemaphoreTake(state_lock, portMAX_DELAY);
    int64_t deadline = timer_deadline_us;
    bool timer_on = timer_turn_on;
    daily_schedule_t current[SCHEDULE_COUNT];
    memcpy(current, schedules, sizeof(current));
    xSemaphoreGive(state_lock);
    int64_t remaining = deadline > now_us ? (deadline - now_us + 999999) / 1000000 : 0;

    snprintf(response, sizeof(response),
             "{\"synced\":%s,\"source\":\"%s\",\"local_time\":\"%s\",\"timezone\":\"%s\","
             "\"timer\":{\"active\":%s,\"action\":\"%s\",\"remaining_seconds\":%lld},"
             "\"schedules\":["
             "{\"enabled\":%s,\"on\":\"%02u:%02u\",\"off\":\"%02u:%02u\"},"
             "{\"enabled\":%s,\"on\":\"%02u:%02u\",\"off\":\"%02u:%02u\"},"
             "{\"enabled\":%s,\"on\":\"%02u:%02u\",\"off\":\"%02u:%02u\"}]}",
             synced ? "true" : "false", clock_source, local_time, LAMP_TIMEZONE,
             remaining > 0 ? "true" : "false", timer_on ? "on" : "off", (long long)remaining,
             current[0].enabled ? "true" : "false", current[0].on_minute / 60, current[0].on_minute % 60,
             current[0].off_minute / 60, current[0].off_minute % 60,
             current[1].enabled ? "true" : "false", current[1].on_minute / 60, current[1].on_minute % 60,
             current[1].off_minute / 60, current[1].off_minute % 60,
             current[2].enabled ? "true" : "false", current[2].on_minute / 60, current[2].on_minute % 60,
             current[2].off_minute / 60, current[2].off_minute % 60);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t automation_post_handler(httpd_req_t *request)
{
    char body[640];
    int received = httpd_req_recv(request, body, sizeof(body) - 1);
    if (received <= 0) return ESP_FAIL;
    body[received] = '\0';

    bool cancel;
    bool schedule_changed = false;
    if (json_bool(body, "\"cancel_timer\"", &cancel) && cancel) {
        xSemaphoreTake(state_lock, portMAX_DELAY);
        timer_deadline_us = 0;
        xSemaphoreGive(state_lock);
    }

    char *minutes_field = strstr(body, "\"timer_minutes\"");
    char *action_field = strstr(body, "\"timer_action\"");
    if (minutes_field != NULL && action_field != NULL) {
        char *separator = strchr(minutes_field, ':');
        long minutes = separator != NULL ? strtol(separator + 1, NULL, 10) : 0;
        char action[4];
        separator = strchr(action_field, ':');
        if (minutes < 1 || minutes > 1440 || separator == NULL ||
            sscanf(separator + 1, " \"%3[^\"]\"", action) != 1 ||
            (strcmp(action, "on") != 0 && strcmp(action, "off") != 0)) {
            httpd_resp_set_status(request, "400 Bad Request");
            return httpd_resp_sendstr(request, "Invalid timer");
        }
        xSemaphoreTake(state_lock, portMAX_DELAY);
        timer_turn_on = strcmp(action, "on") == 0;
        timer_deadline_us = esp_timer_get_time() + (int64_t)minutes * 60 * 1000000;
        xSemaphoreGive(state_lock);
    }

    xSemaphoreTake(state_lock, portMAX_DELAY);
    for (size_t i = 0; i < SCHEDULE_COUNT; i++) {
        char key[32];
        bool enabled;
        uint16_t minute;
        snprintf(key, sizeof(key), "\"schedule_%u_enabled\"", (unsigned)(i + 1));
        if (json_bool(body, key, &enabled)) {
            schedules[i].enabled = enabled;
            schedule_changed = true;
        }
        snprintf(key, sizeof(key), "\"schedule_%u_on\"", (unsigned)(i + 1));
        if (json_time(body, key, &minute)) {
            schedules[i].on_minute = minute;
            schedule_changed = true;
        }
        snprintf(key, sizeof(key), "\"schedule_%u_off\"", (unsigned)(i + 1));
        if (json_time(body, key, &minute)) {
            schedules[i].off_minute = minute;
            schedule_changed = true;
        }
    }
    xSemaphoreGive(state_lock);
    if (schedule_changed && schedule_save() != ESP_OK) {
        httpd_resp_set_status(request, "500 Internal Server Error");
        return httpd_resp_sendstr(request, "Could not save schedule");
    }

    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, "{\"ok\":true}");
}

static void mdns_start(void)
{
    esp_err_t result = mdns_init();
    if (result == ESP_OK) {
        result = mdns_hostname_set(LAMP_HOSTNAME);
    }
    if (result == ESP_OK) {
        result = mdns_instance_name_set("ESP32 Desk Lamp");
    }
    if (result == ESP_OK) {
        result = mdns_service_add(NULL, "_http", "_tcp", LAMP_HTTP_PORT, NULL, 0);
    }

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "mDNS ready: http://%s.local/", LAMP_HOSTNAME);
    } else {
        ESP_LOGW(TAG, "mDNS setup failed: %s; use the numeric IP address", esp_err_to_name(result));
        mdns_free();
    }
}

static void ota_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static bool next_schedule_delay(uint64_t *delay_seconds)
{
    if (!clock_is_synced()) return false;

    daily_schedule_t current[SCHEDULE_COUNT];
    xSemaphoreTake(state_lock, portMAX_DELAY);
    memcpy(current, schedules, sizeof(current));
    xSemaphoreGive(state_lock);

    time_t now;
    struct tm local;
    time(&now);
    localtime_r(&now, &local);
    int now_second = local.tm_hour * 3600 + local.tm_min * 60 + local.tm_sec;
    uint64_t nearest = UINT64_MAX;
    for (size_t i = 0; i < SCHEDULE_COUNT; i++) {
        if (!current[i].enabled || current[i].on_minute == current[i].off_minute) continue;
        int candidate = current[i].on_minute * 60 - now_second;
        if (candidate <= 0) candidate += 24 * 60 * 60;
        if ((uint64_t)candidate < nearest) nearest = (uint64_t)candidate;
    }
    if (nearest == UINT64_MAX) return false;
    *delay_seconds = nearest;
    return true;
}

static void deep_sleep_task(void *arg)
{
    uint64_t wakeup_us = *(uint64_t *)arg;
    free(arg);
    vTaskDelay(pdMS_TO_TICKS(LAMP_FADE_TIME_MS + 500));
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_us));
    ESP_LOGI(TAG, "entering deep sleep for %llu seconds", (unsigned long long)(wakeup_us / 1000000));
    esp_deep_sleep_start();
}

static esp_err_t sleep_post_handler(httpd_req_t *request)
{
    uint64_t delay_seconds;
    if (!next_schedule_delay(&delay_seconds)) {
        httpd_resp_set_status(request, "400 Bad Request");
        return httpd_resp_sendstr(request, "Synchronize the clock and enable at least one valid schedule first");
    }

    set_mode("off");
    esp_err_t save_result = state_save();
    if (save_result != ESP_OK) {
        ESP_LOGW(TAG, "could not save off state before deep sleep: %s", esp_err_to_name(save_result));
    }

    uint64_t *wakeup_us = malloc(sizeof(*wakeup_us));
    if (wakeup_us == NULL) {
        httpd_resp_set_status(request, "500 Internal Server Error");
        return httpd_resp_sendstr(request, "Could not prepare deep sleep");
    }
    *wakeup_us = delay_seconds * 1000000ULL;
    if (xTaskCreate(deep_sleep_task, "deep_sleep", 2048, wakeup_us, 5, NULL) != pdPASS) {
        free(wakeup_us);
        ESP_LOGE(TAG, "could not create deep sleep task");
        httpd_resp_set_status(request, "500 Internal Server Error");
        return httpd_resp_sendstr(request, "Could not start deep sleep");
    }
    char response[64];
    snprintf(response, sizeof(response), "{\"ok\":true,\"wake_seconds\":%llu}",
             (unsigned long long)delay_seconds);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, response);
}

static esp_err_t ota_error(httpd_req_t *request, const char *message)
{
    ESP_LOGE(TAG, "OTA update failed: %s", message);
    httpd_resp_set_status(request, "500 Internal Server Error");
    httpd_resp_set_type(request, "text/plain");
    return httpd_resp_sendstr(request, message);
}

static esp_err_t ota_post_handler(httpd_req_t *request)
{
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL) {
        return ota_error(request, "No OTA partition is available");
    }
    if (request->content_len <= 0 || (size_t)request->content_len > partition->size) {
        httpd_resp_set_status(request, "400 Bad Request");
        return httpd_resp_sendstr(request, "Firmware file is empty or too large");
    }

    ESP_LOGI(TAG, "receiving %d-byte OTA image into %s", request->content_len, partition->label);
    esp_ota_handle_t ota_handle = 0;
    esp_err_t result = esp_ota_begin(partition, request->content_len, &ota_handle);
    if (result != ESP_OK) {
        return ota_error(request, esp_err_to_name(result));
    }

    char buffer[1024];
    int remaining = request->content_len;
    int timeout_count = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(request, buffer,
                                      remaining < (int)sizeof(buffer) ? remaining : (int)sizeof(buffer));
        if (received == HTTPD_SOCK_ERR_TIMEOUT && timeout_count++ < 5) {
            continue;
        }
        if (received <= 0) {
            esp_ota_abort(ota_handle);
            return ota_error(request, "Firmware upload was interrupted");
        }
        timeout_count = 0;
        result = esp_ota_write(ota_handle, buffer, received);
        if (result != ESP_OK) {
            esp_ota_abort(ota_handle);
            return ota_error(request, esp_err_to_name(result));
        }
        remaining -= received;
    }

    result = esp_ota_end(ota_handle);
    if (result != ESP_OK) {
        return ota_error(request, esp_err_to_name(result));
    }
    result = esp_ota_set_boot_partition(partition);
    if (result != ESP_OK) {
        return ota_error(request, esp_err_to_name(result));
    }

    ESP_LOGI(TAG, "OTA update installed; restarting");
    httpd_resp_set_type(request, "text/plain");
    esp_err_t response_result = httpd_resp_sendstr(request, "Update installed");
    if (xTaskCreate(ota_restart_task, "ota_restart", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not create OTA restart task; restart manually");
    }
    return response_result;
}

static void http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = LAMP_HTTP_PORT;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
    httpd_uri_t state_get_uri = {.uri = "/api/state", .method = HTTP_GET, .handler = state_get_handler};
    httpd_uri_t state_post_uri = {.uri = "/api/state", .method = HTTP_POST, .handler = state_post_handler};
    httpd_uri_t automation_get_uri = {.uri = "/api/automation", .method = HTTP_GET, .handler = automation_get_handler};
    httpd_uri_t automation_post_uri = {.uri = "/api/automation", .method = HTTP_POST, .handler = automation_post_handler};
    httpd_uri_t sleep_post_uri = {.uri = "/api/sleep", .method = HTTP_POST, .handler = sleep_post_handler};
    httpd_uri_t ota_post_uri = {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_post_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &state_get_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &state_post_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &automation_get_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &automation_post_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &sleep_post_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_post_uri));
}

void lamp_init(void)
{
    state_lock = xSemaphoreCreateMutex();
    pwm_init();
    set_mode("off");

    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    state_load();
    if (restore_on_boot &&
        xTaskCreate(restore_state_task, "restore_state", 2048, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not restore saved lamp state");
    }
    schedule_load();
    rtc_start();

    wifi_init();
    clock_start();
    mdns_start();
    http_server_start();
    if (xTaskCreate(automation_task, "automation", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not start timer and schedule task");
    }
    ESP_LOGI(TAG, "lamp control server started at http://%s.local/", LAMP_HOSTNAME);
}
