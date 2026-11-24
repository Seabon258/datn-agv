# Published by stm32_bridge on /motor_state. Wheel order everywhere: [FL, FR, RL, RR].
std_msgs/Header header
float32[4] rpm_target
float32[4] rpm_actual
int16[4] pwm_permille        # -1000 .. 1000
uint8 encoder_fault_mask     # bit0 FL, bit1 FR, bit2 RL, bit3 RR
