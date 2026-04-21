CurrentSense_t current_sense;

void CurrentSense_Disable(CurrentSense_t *cs)
{
    if (!cs) return;
    cs->enabled = 0;
}