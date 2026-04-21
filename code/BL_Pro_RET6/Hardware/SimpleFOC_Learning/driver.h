typedef struct {
    uint8_t enabled;
    TIM_HandleTypeDef *htim;
    uint32_t chA;
    uint32_t chB;
    uint32_t chC;       
} Driver_t;



