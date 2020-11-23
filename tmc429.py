import pyb
import time

SMDA_M0               = const(0)            #select motor 1 setting
SMDA_M1               = const(1)            #select motor 2 setting
SMDA_M2               = const(2)            #select motor 3 setting
SMDA_COMMON           = const(3)            #select goalb setting
RRS_RAM               = const(1)            #select get from RAM
RRS_REG               = const(0)            #select get from register
RW_WRITE              = const(0)            #select write mode
RW_READ               = const(1)            #select read mode 
version_addr          = const(0b1001)       #address of version
if_config_addr        = const(0b0100)
smgp_addr             = const(0b1111)
pulsediv_rampdiv_addr = const(0b1100)
refconf_rm_addr       = const(0b1010)
vmin_addr             = const(0b0010)
vmax_addr             = const(0b0011)
x_target_addr         = const(0b0000)
x_actual_addr         = const(0b0001)
v_target_addr         = const(0b0100)
v_actual_addr         = const(0b0101)
amax_addr             = const(0b0110)
a_actual_addr         = const(0b0111)
switch_flag_addr      = const(0b1110)
pdiv_pmul_addr        = const(0b1001)  


clk = pyb.Pin('PA8',pyb.Pin.AF_PP,af=0)

class TMC429:
    def __init__(self,spi_num=1,cs_pin=None,amax=128,motor=None):
        self.spi = pyb.SPI(spi_num,pyb.SPI.MASTER,prescaler=256,phase=1,polarity=1)
        self.tmc_cs = cs_pin
        self.tmc_cs.high()

        self.Motor = motor

        self.write_reg(if_config_addr,0x0122,SMDA_COMMON)
        self.write_reg(smgp_addr,0x000700,SMDA_COMMON)
        self.write_reg(vmin_addr,1,self.Motor)
        # self.write_reg(vmin_addr,1,SMDA_M1)
        # self.write_reg(vmin_addr,1,SMDA_M2)
        self.write_reg(vmax_addr,200,self.Motor)
        # self.write_reg(vmax_addr,2047,SMDA_M1)
        # self.write_reg(vmax_addr,2047,SMDA_M2)
        self.write_reg(amax_addr,amax,self.Motor)
        # self.write_reg(amax_addr,amax,SMDA_M1)
        # self.write_reg(amax_addr,amax,SMDA_M2)
        self.write_reg(pdiv_pmul_addr,0x1007,self.Motor)

        self.EnableLeafLimit(1)
        self.EnableRightLimit(1)
        self.VelocityMode()
        self.setPulseDiv(0x2306)
        self.isHome=2



    def write_reg(self,reg_addr,data,SMDA):
        self.tmc_cs.low()
        reg_val=[0,0,0,0]
        reg_addr = (SMDA << 5) | (reg_addr << 1) | RW_WRITE
        reg_val[0]=self.spi.send_recv(reg_addr)
        reg_val[1]=self.spi.send_recv(0xFF&(data>>16))
        reg_val[2]=self.spi.send_recv(0xFF&(data>>8))
        reg_val[3]=self.spi.send_recv(0xFF&data)
        self.tmc_cs.high()

    def read_reg(self,regaddr,SMDA):
        reg_val=[0,0,0,0]
        regaddr = (SMDA << 5) | (regaddr << 1) | RW_READ 

        self.tmc_cs.low()
        reg_val[0]=self.spi.send_recv(regaddr)
        reg_val[1]=self.spi.send_recv(0)
        reg_val[2]=self.spi.send_recv(0)
        reg_val[3]=self.spi.send_recv(0)
        # version = ord(reg_val[1])<<16 | ord(reg_val[2])<<8 | ord(reg_val[3])
        self.tmc_cs.high()
        return ord(reg_val[1])<<16 | ord(reg_val[2])<<8 | ord(reg_val[3])

    def setPulseDiv(self,data):
        self.write_reg(pulsediv_rampdiv_addr,data,self.Motor)

    def EnableLeafLimit(self,state):
        if state:
            data = self.read_reg(refconf_rm_addr,self.Motor) & 0xfffeff
            self.write_reg(refconf_rm_addr,data,self.Motor)
        else:
            data = self.read_reg(refconf_rm_addr,self.Motor) | 0X000100
            self.write_reg(refconf_rm_addr,data,self.Motor)

    def EnableRightLimit(self,state):
        if state:
            data = self.read_reg(refconf_rm_addr,self.Motor) & 0xfffdff
            self.write_reg(refconf_rm_addr,data,self.Motor)
        else:
            data = self.read_reg(refconf_rm_addr,self.Motor) | 0X000200
            self.write_reg(refconf_rm_addr,data,self.Motor)

    def VelocityMode(self):
        data = self.read_reg(refconf_rm_addr,self.Motor) & 0xfffff0 | 0x000002
        self.write_reg(refconf_rm_addr,data,self.Motor)

    def positionMode(self,velocity,position):
        # self.setVelocity(velocity)
        data = self.read_reg(refconf_rm_addr,self.Motor) & 0xfffff0
        self.write_reg(refconf_rm_addr,data,self.Motor)
        self.write_reg(x_target_addr,int(position),self.Motor)

    def setVelocity(self,velocity):
        self.write_reg(v_target_addr,velocity,self.Motor)

    def readSWL(self):
        return self.read_reg(switch_flag_addr,3) & (2**(2*self.Motor + 1)) and 1

    def readSWR(self):
        return self.read_reg(switch_flag_addr,3) & (2**(2*self.Motor)) and 1

    def home(self,velocity):
        self.isHome=1
        self.velocity=velocity
        self.readSWL() # 初次读取寄存器值，刷新
        self.setVelocity(velocity)
    
    def getVelocityStatus(self):
        if self.isHome==2:
            return 2

        elif self.isHome==1:
            if self.readSWL()==1:
                self.write_reg(x_actual_addr,0,self.Motor)
                self.setVelocity(0)
                self.isHome=0
            return 1

        elif self.isHome==0:
            if (self.read_reg(v_actual_addr,self.Motor)):
                return 1
            else:
                return 0

    def readPosition(self):
        return self.read_reg(x_actual_addr,self.Motor)

    def stopMove(self):
        self.setVelocity(0)

    def test_home(self):
        data = self.read_reg(refconf_rm_addr,self.Motor) | 0x100
        self.write_reg(refconf_rm_addr,data,self.Motor)
        self.readSWL()
        self.setVelocity(-100)


