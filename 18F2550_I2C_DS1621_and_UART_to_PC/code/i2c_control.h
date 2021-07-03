void initialize_i2c(void);
void start_i2c(void);
void i2c_repeated_start(void);
void i2c_stop(void);
char i2c_write_byte(unsigned char m_data);
unsigned char i2c_read_byte(void);
void send_master_ack(void);
void send_master_nack(void);
void i2c_wait(void);