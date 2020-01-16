#include <stdint.h>

#include <rthw.h>
#include <rtthread.h>

//@HRM The read and write test of the RAM zone
void mem_test(uint32_t address, uint32_t size )
{
    uint32_t i;
    //@HRM Print the address and size first
    rt_kprintf("memtest,address: 0x%08X size: 0x%08X\r\n", address, size);

    /**< 8bit test */
    {
        //@HRM 8 bit address write
        uint8_t * p_uint8_t = (uint8_t *)address;
        for(i=0; i<size/sizeof(uint8_t); i++)
        {
            *p_uint8_t++ = (uint8_t)i;
        }
        //@HRM 8 bit address read
        p_uint8_t = (uint8_t *)address;
        for(i=0; i<size/sizeof(uint8_t); i++)
        {
            if( *p_uint8_t != (uint8_t)i )
            {
                //@HRM if read and write are not the same, then system halt
                rt_kprintf("8bit test fail @ 0x%08X\r\nsystem halt!!!!!",(uint32_t)p_uint8_t);
                while(1);
            }
            p_uint8_t++;
        }
        rt_kprintf("8bit test pass!!\r\n");
    }

    /**< 16bit test */
    {
        //@HRM similar to 8 bit eaxmple
        uint16_t * p_uint16_t = (uint16_t *)address;
        for(i=0; i<size/sizeof(uint16_t); i++)
        {
            *p_uint16_t++ = (uint16_t)i;
        }

        p_uint16_t = (uint16_t *)address;
        for(i=0; i<size/sizeof(uint16_t); i++)
        {
            if( *p_uint16_t != (uint16_t)i )
            {
                rt_kprintf("16bit test fail @ 0x%08X\r\nsystem halt!!!!!",(uint32_t)p_uint16_t);
                while(1);
            }
            p_uint16_t++;
        }
        rt_kprintf("16bit test pass!!\r\n");
    }

    /**< 32bit test */
    {
        //@HRM similar to 8 bit eaxmple
        uint32_t * p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            *p_uint32_t++ = (uint32_t)i;
        }

        p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            if( *p_uint32_t != (uint32_t)i )
            {
                rt_kprintf("32bit test fail @ 0x%08X\r\nsystem halt!!!!!",(uint32_t)p_uint32_t);
                while(1);
            }
            p_uint32_t++;
        }
        rt_kprintf("32bit test pass!!\r\n");
    }

    /**< 32bit Loopback test */
    {
        //@HRM similar to 8 bit eaxmple
        uint32_t * p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            *p_uint32_t  = (uint32_t)p_uint32_t;
            p_uint32_t++;
        }

        p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            if( *p_uint32_t != (uint32_t)p_uint32_t )
            {
                rt_kprintf("32bit Loopback test fail @ 0x%08X", (uint32_t)p_uint32_t);
                rt_kprintf(" data:0x%08X \r\n", (uint32_t)*p_uint32_t);
                rt_kprintf("system halt!!!!!",(uint32_t)p_uint32_t);
                while(1);
            }
            p_uint32_t++;
        }
        rt_kprintf("32bit Loopback test pass!!\r\n");
    }
}

//@HRM export to finsh console
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(mem_test, mem_test(0xA0000000, 0x00100000)  );
#endif
