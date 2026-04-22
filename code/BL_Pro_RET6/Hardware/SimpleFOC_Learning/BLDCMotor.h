//无刷电机基础参数配置
typedef struct {
    float pole_pairs;
    float phase_resistance;
    float kv;
    float Lq;
    float Ld;
} MotorParam;


typedef struct
{
    float Ua, Ub, Uc;
}PhaseVoltage;

typedef struct{
    MotorParam Motor_Param;
    Driver_t *driver;
    CurrentSense_t *current_sense;
    PhaseVoltage Motor_PhaseVoltage;
}Motor_t;