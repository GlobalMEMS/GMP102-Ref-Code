Quick Porting Guide
===================
Add the following codes for your platform and you are pretty much ready to go.

sensor_driver_test.c: main program
----------------------------------

* Add the time delay function
    ```
    #define DELAY_MS(ms)	//.....     /* Add your time delay function here */
    ```

* Add HW initialization
    ```
    /* Add your HW initialization code here
    ...
    ...
    ...
    ...
    */
    ```

bus_support.c
-------------

* Add I2C read/write function pointer to your I2C functions
    ```
	 pbus->bus_read = I2C_read_bytes;    /* Put your I2C read function pointer here */
	 pbus->bus_write = I2C_write_bytes;  /* Put your I2C write function pointer here */
    ```

  Your I2C read/write functions should implement the following interface:
    ```
    //******************************************************************************
    //
    //! @brief Read multiple bytes from I2C slave with address devAddr
    //!
    //! @param devAddr 7-bit device slave address
    //! @param regAddr start register address
    //! @param dataBuf data byte array to store the reading
    //! @param len number of data to be read
    //
    //! @return number of bytes read
    //
    //******************************************************************************
    char I2C_read_bytes(unsigned char devAddr, 
    					unsigned char regAddr, 
    					unsigned char* dataBuf, 
    					unsigned char len)
     {
      /* ..... */
     }
    
    //******************************************************************************
    //
    //! @brief Write multiple bytes to I2C slave with address devAddr
    //!
    //! @param devAddr 7-bit device slave address
    //! @param regAddr register address
    //! @param dataBuf data bytes array buffer with datas to write
    //! @param len number of bytes to write
    //!
    //! @param return number of byte written
    //
    //******************************************************************************
    char I2C_write_bytes(unsigned char devAddr,
                         unsigned char regAddr,
    					 unsigned char* dataBuf,
    					 unsigned char len)
    
    {
     /* ..... */
    }
    ```
   
