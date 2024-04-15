#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <espFoC/esp_foc.h>

struct motor_hardware_stub stub;
struct esp_foc_motor_interface *hw;
struct esp_foc_motor_control control;
int should_stop = 0;

using namespace std;

int main(int argc, char **argv)
{
    int err;

    err = motor_hardware_stub_init(&stub, &hw);
    if(err)
        cout << "failed to initialize the motor hardware err:" << err << endl;

    err = esp_foc_init_controller(&control, hw);
    if(err)
        cout << "failed to initialize esp foc controller err:" << err << endl;

    err = esp_foc_controller_set_speed(&control, 360.0f);
    if(err)
        cout << "failed to set the initial speed err" << err << endl;

    while(!should_stop) {
        err = esp_foc_controller_run(&ctl);
        if(err)
            cout << "error when running controller err:" << err << endl;
        usleep(1000);
    }
}
