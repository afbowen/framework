#include "driver_invensense.h"
#include "invensense_internal.h"

#include <modules/uavcan_debug/uavcan_debug.h>

static void invensense_read(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf);
static void invensense_write(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf);


static float invensense_get_temp(struct invensense_instance_s* instance);
static gyro_data_t invensense_get_gyro(struct invensense_instance_s* instance);
static accel_data_t invensense_get_accel(struct invensense_instance_s* instance);

static void invensense_print_accel_offsets(struct invensense_instance_s* instance);
static void invensense_print_sample_rate_divider(struct invensense_instance_s* instance);
static void invensense_fifo_enable(struct invensense_instance_s* instance, fifo_setting_t fifo_setting);
static void invensense_fifo_disable(struct invensense_instance_s* instance);

void invensense_device_reset(struct invensense_instance_s* instance);

accel_scale_t accel_scale;
gyro_scale_t gyro_scale;


void invensense_init(struct invensense_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum invensense_imu_type_t imu_type, accel_scale_t accel_scale_g, gyro_scale_t gyro_scale_dps) {
    if (!instance) {
        return;
    }
    
    //Intialize SPI device
    spi_device_init(&instance->spi_dev, spi_idx, select_line, 10000, 8, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);
    
    uint8_t whoami = 0;
    switch(imu_type)
    {
        case INVENSENSE_IMU_TYPE_ICM20602:
            
            //Verify device by checking whoami register
            invensense_read(instance, INVENSENSE_REG_WHO_AM_I, sizeof(uint8_t), &whoami);
            uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "whoami = 0x%02X", whoami);
            
            if(whoami == 0x12){
                spi_device_set_max_speed_hz(&instance->spi_dev, 10000000);  //If the expected device is connected, set the baud for 10MHz
            }
            else{
                uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "Device not found"); 
            }
            break;
            
        default:
            uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "Invalid IMU type");
            break;
    }
    
    
    //Reset device (before configuring)
    invensense_device_reset(instance);
    
    //Setup device config
    uint8_t reg_config_current = 0;                                                                         //DEBUG
    invensense_read(instance, INVENSENSE_REG_CONFIG, sizeof(uint8_t), &reg_config_current);                 //DEBUG
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "config_before = 0x%02x", reg_config_current);     //DEBUG
    uint8_t reg_config_out = 0x00;
    reg_config_out = BIT_DEVICE_DISABLE;    //FIFO will overwrite oldest data with newest data if buffer filled
    invensense_write(instance, INVENSENSE_REG_CONFIG, sizeof(uint8_t), &reg_config_out);
    invensense_read(instance, INVENSENSE_REG_CONFIG, sizeof(uint8_t), &reg_config_current);                 //DEBUG
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "config_after = 0x%02x", reg_config_current);      //DEBUG
   
    //Setup power management
uint8_t reg_pwr1_current = 0;                                                                               //DEBUG
    invensense_read(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &reg_pwr1_current);               //DEBUG
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "pwr_before = 0x%02x", reg_pwr1_current);          //DEBUG
    uint8_t reg_pwr1_out = 0x00;
    invensense_write(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &reg_pwr1_out); 
    invensense_read(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &reg_pwr1_current);               //DEBUG
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "pwr_after = 0x%02x", reg_pwr1_current);           //DEBUG

        
    //Configure gyro and accel
    uint8_t gyro_config_out = 0;
    uint8_t accel_config_out = 0;

    switch(gyro_scale_dps)
    {
        case GYRO_FS_250dps:
            gyro_config_out = BIT_GYRO_FS_250dps;   //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_250dps;    //Set global gyro scale variable to relfect register

        case GYRO_FS_500dps:
            gyro_config_out = BIT_GYRO_FS_500dps;   
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_500dps;  
        
        case GYRO_FS_1000dps:
            gyro_config_out = BIT_GYRO_FS_1000dps;   
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_1000dps;   

       case GYRO_FS_2000dps:
            gyro_config_out = BIT_GYRO_FS_2000dps;   
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_2000dps;    
            
            break;
        default:
            break;
    }
    
    
    switch (accel_scale_g)
    {
        case ACCEL_FS_2g:
            accel_config_out = BIT_ACCEL_FS_2g;     //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_2g;      //set global accel scale variable to reflect register
            break;
            
        case ACCEL_FS_4g:
            accel_config_out = BIT_ACCEL_FS_4g;     
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_4g;      
            break;
            
        case ACCEL_FS_8g:
            accel_config_out = BIT_ACCEL_FS_8g;     //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_8g;     
            break;
        
        case ACCEL_FS_16g:
            accel_config_out = BIT_ACCEL_FS_16g;     //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_16g;    
            break;
            
        default:
            break;
    }
    
    
    
    
    invensense_print_accel_offsets(instance);
    invensense_print_sample_rate_divider(instance);

    
    //Get temperature
    float temperature = invensense_get_temp(instance);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "temperature %f", temperature);
    
    //Read gyro
    gyro_data_t gyro_data = invensense_get_gyro(instance);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "gyro_x %f", gyro_data.x);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "gyro_y %f", gyro_data.y);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "gyro_z %f", gyro_data.z);
    
    //Read accel
    accel_data_t accel_data = invensense_get_accel(instance);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "accel_x %f", accel_data.x);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "accel_y %f", accel_data.y);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "accel_z %f", accel_data.z);
}
/////////////
//FUNCTIONS//
/////////////
//READ
static void invensense_read(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf) 
{
    reg |= 0x80;
    
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    spi_device_receive(&instance->spi_dev, len, buf);
    spi_device_end(&instance->spi_dev);
}


//WRITE
static void invensense_write(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf)
{
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg); 
    spi_device_send(&instance->spi_dev, len, buf); 
    spi_device_end(&instance->spi_dev);
}


///////////////
//SENSOR DATA//
///////////////

//READ TEMP
float invensense_get_temp(struct invensense_instance_s* instance)
{
    
    uint8_t temp_reg[2];
    invensense_read(instance, INVENSENSE_REG_TEMP_OUT_H, sizeof(uint16_t), temp_reg);    //should read two bytes into temp[0] and temp[1] (?)
    uint16_t temperature = (temp_reg[1] << 8) | (temp_reg[0]);
    return (temperature/326.8 + 25);
}


//READ GYRO (gyro_t)
static gyro_data_t invensense_get_gyro(struct invensense_instance_s* instance)
{
    uint8_t reg_gyro[6];
    invensense_read(instance, INVENSENSE_REG_GYRO_XOUT_H, 6, reg_gyro);
    
    int16_t data_x = (reg_gyro[0] << 8)|(reg_gyro[1]);
    int16_t data_y = (reg_gyro[2] << 8)|(reg_gyro[3]);
    int16_t data_z = (reg_gyro[4] << 8)|(reg_gyro[5]);
    
    gyro_data_t data_out;   
    switch(gyro_scale)
    {
        case GYRO_FS_250dps:
            data_out.x = (float)250*data_x/32767;
            data_out.y = (float)250*data_y/32767;
            data_out.z = (float)250*data_z/32767;
            break;
            
        case GYRO_FS_500dps:
            data_out.x = (float)500*data_x/32767;
            data_out.y = (float)500*data_y/32767;
            data_out.z = (float)500*data_z/32767;
            break;
        
        case GYRO_FS_1000dps:
            data_out.x = (float)1000*data_x/32767;
            data_out.y = (float)1000*data_y/32767;
            data_out.z = (float)1000*data_z/32767;
            break;
            
        case GYRO_FS_2000dps:
            data_out.x = (float)2000*data_x/32767;
            data_out.y = (float)2000*data_y/32767;
            data_out.z = (float)2000*data_z/32767;
            break;
        
        default:
            break;
            
    }
    return data_out;
}

//READ ACCEL (accel_t)
static accel_data_t invensense_get_accel(struct invensense_instance_s* instance)
{
    uint8_t reg_accel[6];
    invensense_read(instance, INVENSENSE_REG_ACCEL_XOUT_H, 6, reg_accel);
    
    int16_t data_x = (reg_accel[0] << 8)|(reg_accel[1]);
    int16_t data_y = (reg_accel[2] << 8)|(reg_accel[3]);
    int16_t data_z = (reg_accel[4] << 8)|(reg_accel[5]);
    
    accel_data_t data_out;   
    switch(accel_scale)
    {
        case ACCEL_FS_2g:
            data_out.x = (float)2*data_x/32767;
            data_out.y = (float)2*data_y/32767;
            data_out.z = (float)2*data_z/32767;
            break;
            
        case ACCEL_FS_4g:
            data_out.x = (float)4*data_x/32767;
            data_out.y = (float)4*data_y/32767;
            data_out.z = (float)4*data_z/32767;
            break;
            
        case ACCEL_FS_8g:
            data_out.x = (float)8*data_x/32767;
            data_out.y = (float)8*data_y/32767;
            data_out.z = (float)8*data_z/32767;
            break;
            
        case ACCEL_FS_16g:
            data_out.x = (float)16*data_x/32767;
            data_out.y = (float)16*data_y/32767;
            data_out.z = (float)16*data_z/32767;
            break;
            
        default:
            break;        
    }        
    return data_out;
}

////////
//FIFO//
////////

//DISABLE FIFO
static void invensense_fifo_disable(struct invensense_instance_s* instance)
{
    uint8_t reg_user_ctrl_out = 0x00;
    invensense_write(instance, INVENSENSE_REG_USER_CTRL, sizeof(uint8_t), &reg_user_ctrl_out);
}
    

//ENABLE FIFO
static void invensense_fifo_enable(struct invensense_instance_s* instance, fifo_setting_t fifo_setting)
{
    uint8_t reg_user_ctrl_out = 0x00;
    reg_user_ctrl_out = BIT_FIFO_EN;
    invensense_write(instance, INVENSENSE_REG_USER_CTRL, sizeof(uint8_t), &reg_user_ctrl_out);
    
    uint8_t reg_fifo_en_out = 0x00;
    switch(fifo_setting)
    {
        case ACCEL_FIFO_EN:
            reg_fifo_en_out = BIT_ACCEL_FIFO_EN;
            break;
        
        case GYRO_FIFO_EN:
            reg_fifo_en_out = BIT_GYRO_FIFO_EN;
            break;
        
        case ACCEL_GYRO_FIFO_EN:
            reg_fifo_en_out = BIT_ACCEL_FIFO_EN | BIT_GYRO_FIFO_EN;
            break;
        
        default:
            break; 
    }
    
    invensense_write(instance, INVENSENSE_REG_FIFO_EN, sizeof(uint8_t), &reg_fifo_en_out);
}

//READ FIFO COUNT
static uint16_t invensense_get_fifo_count(struct invensense_instance_s* instance, uint8_t enabled)
{
    uint8_t reg_fifo_count[2];
    invensense_read(instance, INVENSENSE_REG_FIFO_COUNTH, 2, reg_fifo_count);
    uint16_t fifo_count = (reg_fifo_count[0] << 8)|(reg_fifo_count[1]);
    return fifo_count;
}



//READ INTERRUPT STATUS
static uint8_t invensense_get_interrupt_status(struct invensense_instance_s* instance)
{
    uint8_t reg_int_status = 0;
    invensense_read(instance, INVENSENSE_REG_SMPLRT_DIV, sizeof(uint8_t), &reg_int_status);
    return reg_int_status;
}

//RESET DEVICE REGISTERS
void invensense_device_reset(struct invensense_instance_s* instance)
{
    uint8_t pwr_mgmt_1_reg = 0;
    invensense_read(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &pwr_mgmt_1_reg);
    uint8_t pwr_mgmt_1_reg_reset = (pwr_mgmt_1_reg | BIT_DEVICE_RESET);
    invensense_write(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &pwr_mgmt_1_reg_reset); 
}

/////////
//DEBUG//
/////////
static void invensense_print_accel_offsets(struct invensense_instance_s* instance)
{
    uint8_t reg_accel_offset[6];
    invensense_read(instance, INVENSENSE_REG_ACCEL_XOUT_H, 6, reg_accel_offset);
    
    int16_t accel_offset_x = (reg_accel_offset[0] << 8)|(reg_accel_offset[1]);
    int16_t accel_offset_y = (reg_accel_offset[2] << 8)|(reg_accel_offset[3]);
    int16_t accel_offset_z = (reg_accel_offset[4] << 8)|(reg_accel_offset[5]);
    
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "accel_offset_x = %d", accel_offset_x);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "accel_offset_y = %d", accel_offset_y);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "accel_offset_z = %d", accel_offset_z);
}
    

static void invensense_print_sample_rate_divider(struct invensense_instance_s* instance)
{
    uint8_t reg_sample_rate_divider;
    invensense_read(instance, INVENSENSE_REG_SMPLRT_DIV, sizeof(uint8_t), &reg_sample_rate_divider);
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "smplrt_div = 0x%02x", reg_sample_rate_divider);
}






 