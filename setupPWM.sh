#! /bin/bash

export ehrpwm1=/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2
#P8.34:
config-pin P8.34 pwm
echo 0 > ${ehrpwm1}/export || true
echo 20000 > ${ehrpwm1}/pwm0/period ; echo 10000 > ${ehrpwm1}/pwm0/duty_cycle
echo 1 > ${ehrpwm1}/pwm0/enable

#P8.36:
config-pin P8.36 pwm
echo 1 > ${ehrpwm1}/export || true
echo 20000 > ${ehrpwm1}/pwm1/period ; echo 10000 > ${ehrpwm1}/pwm1/duty_cycle
echo 1 > ${ehrpwm1}/pwm1/enable

export ehrpwm2=/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip6
#P8.45:
config-pin P8.45 pwm
echo 0 > ${ehrpwm2}/export || true
echo 20000 > ${ehrpwm2}/pwm0/period ; echo 10000 > ${ehrpwm2}/pwm0/duty_cycle
echo 1 > ${ehrpwm2}/pwm0/enable

#P8.46:
config-pin P8.46 pwm
echo 1 > ${ehrpwm2}/export || true
echo 20000 > ${ehrpwm2}/pwm1/period ; echo 10000 > ${ehrpwm2}/pwm1/duty_cycle
echo 1 > ${ehrpwm2}/pwm1/enable
