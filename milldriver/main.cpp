// Interactive Test Session for LeafLabs Maple
// Copyright (c) 2010 LeafLabs LLC.
//
// Useful for testing Maple features and troubleshooting.
// Communicates over SerialUSB.

#include <string.h>

#include <wirish/wirish.h>

// ASCII escape character
#define ESC       ((uint8)27)

// Default USART baud rate
#define BAUD     9600

void cmd_motor_drive_test(void);

// -- setup() and loop() ------------------------------------------------------
//
//
void setup() {
    // Set up the LED to blink
    pinMode(BOARD_LED_PIN, OUTPUT);

    Serial1.begin(BAUD);
    Serial2.begin(BAUD);
    Serial3.begin(BAUD);

    SerialUSB.print("> ");
}

#pragma pack(push, 1)
struct message {
    short magic;
    short size;
    short msgtype;
    short correlation;
};

struct error_message : message {
    enum { type = 0x01 };
    char    what[1];
};

struct info_message : message {
    enum { type = 0x02 };
    char    what[1];
};

struct status_message : message {
    enum { type = 0x03 };
    int status;
};

struct test_message : message {
    enum { type = 0x10 };
    int     count;
    char    msg[16];
};

struct start_message : message {
    enum { type = 0x11 };
    unsigned int velocity;
};

struct stop_message : message {
    enum { type = 0x12 };
};

struct setup_message : message {
    enum { type = 0x13 };
};
#pragma pack(pop)

void send_message(message* m)
{
    SerialUSB.write(m, m->size);
}

static char message_buffer[512];

template <typename T>
T* make_message(char* ptr = message_buffer)
{
    T* m = reinterpret_cast<T*>(ptr);
    m->magic = 0x1eaf;
    m->size = sizeof(T);
    m->msgtype = T::type;

    return m;
}

void send_error(const char* msg)
{
    error_message* m = make_message<error_message>();
    m->size = sizeof(message) + strlen(msg);
    strcpy(m->what, msg);

    send_message(m);
}

void send_info(const char* msg)
{
    info_message* m = make_message<info_message>();
    m->size = sizeof(message) + strlen(msg);
    strcpy(m->what, msg);

    send_message(m);
}

void send_status(int c, short corr)
{
    Serial1.print("status(");
    Serial1.print(c);
    Serial1.print(") corr(");
    Serial1.print(corr);
    Serial1.println(")");

    status_message* m = make_message<status_message>();
    m->status = c;
    m->correlation = corr;
    send_message(m);
}

void test_dispatch(message* msg)
{
    test_message* t = static_cast<test_message*>(msg);

    t->count++;
    strcpy(t->msg, "pong");

    send_message(t);
}

void motor_setup(message* msg);
void motor_start(message* msg);
void motor_stop(message* msg);

typedef void (*dispatch_function)(message* msg);
struct MessageDispatch {
    short msg;
    short size;
    dispatch_function dispatch;
};

MessageDispatch dispatch_table[] = {
    { test_message::type, sizeof(test_message), test_dispatch },
    { setup_message::type, sizeof(setup_message), motor_setup },
    { start_message::type, sizeof(start_message), motor_start },
    { stop_message::type, sizeof(stop_message), motor_stop },
    { -1, 0, 0 }
};

void dispatch_message(message* msg) {
    int dcnt = 0;
    for (int i = 0; dispatch_table[i].msg != -1; i++) {
        if (dispatch_table[i].msg == msg->msgtype) {
            dispatch_table[i].dispatch(msg);
            dcnt++;
        }
    }

    if (dcnt == 0) {
        Serial1.println("not found type");
        Serial1.print(msg->msgtype, 16);
        Serial1.println("\n");
        send_error("unhandled message type");
    }
}

enum ReceiveState {
    RMAGIC1,
    RMAGIC2,
    RSIZE1,
    RSIZE2,
    RTYPE1,
    RTYPE2,
    RCORR1,
    RCORR2,
    RDATA
};


void await_command() {
    static int loopcnt = 0;
    static int lastbyte = 0;
    static char buffer[512];
    static int o = 0;
    static int state = RMAGIC1;
    static int cnt = 0;

    loopcnt++;

    toggleLED();
    if (state > RMAGIC1 && (loopcnt - lastbyte) < 100) {
        delay(10);
    } else {
        state = RMAGIC1;
        o = 0;
        cnt = 0;
        delay(250);
    }

    if (loopcnt % 40 == 0) {
        send_info("alive");
        Serial1.print(".");
    }

    message* pm = reinterpret_cast<message*>(buffer);

    while (SerialUSB.available()) {
        lastbyte = loopcnt;
        buffer[o] = SerialUSB.read();
        //Serial1.print(buffer[o], 16);
        //Serial1.print("/");
        //Serial1.print(buffer[o]);
        //Serial1.print("-");
        //Serial1.print(state);
        //Serial1.println("\n");
        switch (state) {
            case RMAGIC1:
                if (buffer[o] == (char)0xaf) {
                    o++;
                    state = RMAGIC2;
                }
                break;
            case RMAGIC2:
                if (buffer[o] == (char)0x1e) {
                    o++;
                    state = RSIZE1;
                } else {
                    state = RMAGIC1;
                    o = 0;
                }
                break;
            case RSIZE1:
                state = RSIZE2;
                o++;
                break;
            case RSIZE2:
                state = RTYPE1;
                o++;
                cnt = pm->size - sizeof(message);
                //Serial1.print("cnt-");
                //Serial1.print(cnt);
                break;
            case RTYPE1:
                state = RTYPE2;
                o++;
                break;
            case RTYPE2:
                state = RCORR1;
                o++;
                break;
            case RCORR1:
                state = RCORR2;
                o++;
                break;
            case RCORR2:
                state = RDATA;
                o++;
                break;
            case RDATA:
                o++;
                cnt--;
                if (cnt == 0) {
                    Serial1.println("done");
                    dispatch_message(pm);
                    state = RMAGIC1;
                    o = 0;
                }
                break;
        }
    }
}

struct stepper_info {
    timer_dev* timer;
    uint8      timer_channel;
    uint8      step_pin;
    uint8      dir_pin;
};

const double ck_int = CYCLES_PER_MICROSECOND * 1000000.0;
const double pitch = 4.0;
const double microstep = 8; // steps/step 8,4,2,1
const double steps_rev = microstep * 200.0;
const double step_ratio = steps_rev / pitch;

void calc_pwm(double fpwm, int32& arr, int32& psc, int32& ccr) {
    double A = ck_int / fpwm;

    psc = (int)((A / 0xffff)+1.0);
    arr = (int)(A / psc);
    ccr = arr / 2;
}

volatile int intflag = 0;
void stepper_int()
{
    intflag = 1;
}

void config_stepper(stepper_info* s)
{
    timer_init(s->timer);
    timer_set_mode(s->timer, s->timer_channel, TIMER_PWM);
    pinMode(s->step_pin, PWM);

    timer_pause(s->timer);

    int32 arr;
    int32 psc;
    int32 ccr;
    calc_pwm(20, arr, psc, ccr);

    Serial1.print("psc: ");
    Serial1.println(psc);
    Serial1.print("arr: ");
    Serial1.println(arr);
    Serial1.print("ccr: ");
    Serial1.println(ccr);

    timer_set_prescaler(s->timer, psc);
    timer_set_reload(s->timer, arr);
    timer_set_compare(s->timer, s->timer_channel, ccr);
//    timer_attach_interrupt(s->timer, TIMER_CC1_INTERRUPT, stepper_int);
    timer_generate_update(s->timer);
    timer_resume(s->timer);

}

inline double velocity_to_freq(double v)
{
    return step_ratio * v;
}

void stepper_output(stepper_info* s, double v)
{
    int32 arr;
    int32 psc;
    int32 ccr;
    calc_pwm(velocity_to_freq(v), arr, psc, ccr);

    timer_set_prescaler(s->timer, psc);
    timer_set_reload(s->timer, arr);
    timer_set_compare(s->timer, s->timer_channel, ccr);
    timer_generate_update(s->timer);
}

void stepper_stop(stepper_info* s)
{
    timer_pause(s->timer);
}

static stepper_info xinfo = { TIMER2, 1, 11, 12 };
void motor_setup(message* msg)
{
    Serial1.println("motor setup");
    config_stepper(&xinfo);

    send_status(0, msg->correlation);
}

void motor_start(message* msg)
{
    start_message* sm = reinterpret_cast<start_message*>(msg);
    double v = (sm->velocity / 1000) + ((sm->velocity % 1000) / 1000.0);

    Serial1.print("motor start(");
    Serial1.print(v);
    Serial1.println(")");

    stepper_output(&xinfo, v);

    send_status(0, msg->correlation);
}

void motor_stop(message* msg)
{
    stepper_stop(&xinfo);

    send_status(0, msg->correlation);
}


void cmd_motor_drive_test(void)
{
    stepper_info xinfo = { TIMER2, 1, 11, 12 };

    config_stepper(&xinfo);

    stepper_output(&xinfo, 0);

    SerialUSB.println("any key to stop stepper sweep");
    int    p = 50;
    double v = 0;
    double a = 20;
    double vmax = 100;
    double vmin = 0;
    double d = a * p / 1000.0;
    while (!SerialUSB.available()) {
        delay(p);
        stepper_output(&xinfo, v);
        if (v > vmax || v < vmin) {
            break;
            //d = -d;
        }

        v += d;
    }

    while (!SerialUSB.available()) {
        delay(250);
    }

    stepper_stop(&xinfo);
}


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (1) {
//        loop();
        await_command();
    }
    return 0;
}
