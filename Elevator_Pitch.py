def main():
    import smbus
    import time
    import soundcard
    import numpy as np
    import matplotlib
    # matplotlib.use('GTK3Agg')
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    import RPi.GPIO as GPIO
    import time



    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    PIR_PIN = 4
    GPIO.setup(PIR_PIN, GPIO.IN)


    GPIO.setwarnings(False) # Ignore warning for now
    GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
    GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
    import time



    speaker = soundcard.default_speaker()
    block_size = 524
    samplerate = 44100
    frequency = 440



    class mpu6050:

        # Global Variables
        GRAVITIY_MS2 = 9.80665
        # GRAVITIY_MS2 = 0
        address = None
        bus = None

        # Scale Modifiers
        ACCEL_SCALE_MODIFIER_2G = 16384.0
        ACCEL_SCALE_MODIFIER_4G = 8192.0
        ACCEL_SCALE_MODIFIER_8G = 4096.0
        ACCEL_SCALE_MODIFIER_16G = 2048.0

        GYRO_SCALE_MODIFIER_250DEG = 131.0
        GYRO_SCALE_MODIFIER_500DEG = 65.5
        GYRO_SCALE_MODIFIER_1000DEG = 32.8
        GYRO_SCALE_MODIFIER_2000DEG = 16.4

        # Pre-defined ranges
        ACCEL_RANGE_2G = 0x00
        ACCEL_RANGE_4G = 0x08
        ACCEL_RANGE_8G = 0x10
        ACCEL_RANGE_16G = 0x18

        GYRO_RANGE_250DEG = 0x00
        GYRO_RANGE_500DEG = 0x08
        GYRO_RANGE_1000DEG = 0x10
        GYRO_RANGE_2000DEG = 0x18

        # MPU-6050 Registers
        PWR_MGMT_1 = 0x6B
        PWR_MGMT_2 = 0x6C

        ACCEL_XOUT0 = 0x3B
        ACCEL_YOUT0 = 0x3D
        ACCEL_ZOUT0 = 0x3F

        TEMP_OUT0 = 0x41

        GYRO_XOUT0 = 0x43
        GYRO_YOUT0 = 0x45
        GYRO_ZOUT0 = 0x47

        ACCEL_CONFIG = 0x1C
        GYRO_CONFIG = 0x1B

        def __init__(self, address, bus=1):
            self.address = address
            self.bus = smbus.SMBus(bus)
            # Wake up the MPU-6050 since it starts in sleep mode
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        # I2C communication methods

        def read_i2c_word(self, register):
            # Read the data from the registers
            high = self.bus.read_byte_data(self.address, register)
            low = self.bus.read_byte_data(self.address, register + 1)

            value = (high << 😎 + low

            if (value >= 0x8000):
                return -((65535 - value) + 1)
            else:
                return value

        def set_accel_range(self, accel_range):
            # First change it to 0x00 to make sure we write the correct value later
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

            # Write the new range to the ACCEL_CONFIG register
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

        def read_accel_range(self, raw = False):
            raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

            if raw is True:
                return raw_data
            elif raw is False:
                if raw_data == self.ACCEL_RANGE_2G:
                    return 2
                elif raw_data == self.ACCEL_RANGE_4G:
                    return 4
                elif raw_data == self.ACCEL_RANGE_8G:
                    return 8
                elif raw_data == self.ACCEL_RANGE_16G:
                    return 16
                else:
                    return -1

        def get_accel_data(self, g = False):
            x = self.read_i2c_word(self.ACCEL_XOUT0)
            y = self.read_i2c_word(self.ACCEL_YOUT0)
            z = self.read_i2c_word(self.ACCEL_ZOUT0)

            accel_scale_modifier = None
            accel_range = self.read_accel_range(True)

            if accel_range == self.ACCEL_RANGE_2G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
            elif accel_range == self.ACCEL_RANGE_4G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
            elif accel_range == self.ACCEL_RANGE_8G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
            elif accel_range == self.ACCEL_RANGE_16G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
            else:
                print("Unknown range-accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

            x = x / accel_scale_modifier
            y = y / accel_scale_modifier
            z = z / accel_scale_modifier

            if g is True:
                return {'x': x, 'y': y, 'z': z}
            elif g is False:
                x = x * self.GRAVITIY_MS2
                y = y * self.GRAVITIY_MS2
                z = z * self.GRAVITIY_MS2
                return {'x': x, 'y': y, 'z': z}

        def set_gyro_range(self, gyro_range):
            # First change it to 0x00 to make sure we write the correct value later
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

            # Write the new range to the ACCEL_CONFIG register
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

        def read_gyro_range(self, raw = False):
            raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

            if raw is True:
                return raw_data
            elif raw is False:
                if raw_data == self.GYRO_RANGE_250DEG:
                    return 250
                elif raw_data == self.GYRO_RANGE_500DEG:
                    return 500
                elif raw_data == self.GYRO_RANGE_1000DEG:
                    return 1000
                elif raw_data == self.GYRO_RANGE_2000DEG:
                    return 2000

                else:
                    return -1

        def get_gyro_data(self):
            x = self.read_i2c_word(self.GYRO_XOUT0)
            y = self.read_i2c_word(self.GYRO_YOUT0)
            z = self.read_i2c_word(self.GYRO_ZOUT0)

            gyro_scale_modifier = None
            gyro_range = self.read_gyro_range(True)

            if gyro_range == self.GYRO_RANGE_250DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
            elif gyro_range == self.GYRO_RANGE_500DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
            elif gyro_range == self.GYRO_RANGE_1000DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
            elif gyro_range == self.GYRO_RANGE_2000DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
            else:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

            x = x / gyro_scale_modifier
            y = y / gyro_scale_modifier
            z = self.read_i2c_word(self.GYRO_ZOUT0)

            gyro_scale_modifier = None
            gyro_range = self.read_gyro_range(True)

            if gyro_range == self.GYRO_RANGE_250DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
            elif gyro_range == self.GYRO_RANGE_500DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
            elif gyro_range == self.GYRO_RANGE_1000DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
            elif gyro_range == self.GYRO_RANGE_2000DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
            else:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

            x = x / gyro_scale_modifier
            y = y / gyro_scale_modifier
            z = z / gyro_scale_modifier

            return {'x': x, 'y': y, 'z': z}

        def get_all_data(self):
            temp = self.get_temp()
            accel = self.get_accel_data()
            gyro = self.get_gyro_data()

            return [accel, gyro, temp]

    mpu = mpu6050(0x68)



    def generate_block(block_size, phase_offset, frequency, samplerate):
        time = np.arange(0, block_size) / samplerate
        output_buffer = np.sin(2*np.pi*frequency*time + phase_offset)
        next_phase_offset = 2*np.pi*frequency*block_size/samplerate + phase_offset
        return output_buffer, next_phase_offset

    tries = 5
    delay_buffer = list(np.zeros(100))
    phase_offset = 0


    with speaker.player(samplerate=samplerate, channels=1, blocksize=block_size) as player:

        total = 0
        average = 0
        numReadings = 10
        i = 0;
        total = 0
        stationary = False
        movingup = False
        movingdown = False
        counter = 0
        acceleration_count = []


        x_len = 500 #number of points to display
        y_range = [-20, 30]
        
        #create figure for plotting
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        xs = list(range(0,500))
        ys = [0] * x_len
        ax.set_ylim(y_range)
        
        line, = ax.plot(xs, ys)
        
        plt.title('Acceleration over Time')
        plt.xlabel('Samples')
        plt.ylabel('Acceleration')
        
        
        def animate(i, ys):
            accel_data = mpu.get_accel_data()
            reading = accel_data['z']-9
            ys.append(reading)
                    
            ys = ys[-x_len:]
        
            line.set_ydata(ys)
            
            return line,
            

        
        accel_log = []
        vel_log =[]
        total = 0
        timepassed = 0
        

        while True:

                    
            if GPIO.input(PIR_PIN):
                print('Motion Detected')
                sensor_value = 100

            else:
                print('not dected')
                sensor_value = 10
        


            readings = list(np.zeros(15))
            start = time.time()
            # print (type(readings))
            readings.pop(0)
            # read from the sensor:
            
            
            accel_data = mpu.get_accel_data()
            reading = accel_data['z'] - 9.3
            # reading = int(reading)
            # reading = round(reading, 4)
            readings.append(reading)
            
            acceleration = np.mean(readings)
            acceleration = round(acceleration,4)
            if (-0.01 < acceleration < 0.06): 
                acceleration = 0 
            accel_log.append(acceleration)


            #THE GOD SET UP 
            #THE GOD SET UP 

            if sum(acceleration_count) == 0: 
                if (acceleration > 0.09 or acceleration < -0.02) and (sum(acceleration_count[-100:]) == 0):
                    acceleration_count.append(1)
                else: 
                    acceleration_count.append(0)
            elif sum(acceleration_count) == 1:
                if (acceleration > 0.03 or acceleration < -0.01) and (sum(acceleration_count[-100:]) == 0):
                    acceleration_count.append(1)
                else: 
                    acceleration_count.append(0)

            #THE GOD SET UP
            #THE GOD SET UP 


            
            print("number of accelerations " + str(sum(acceleration_count)))

            ctime = time.time() - start 
            timepassed += ctime
            # ctime = 1.5
            
            velocity = acceleration * (ctime)

            # print(distance)
            # total += velocity 
            total += velocity
            total = round(total, 4)
            # if (-0.001 < total < (0.02 + 0.001*(timepassed))): 
            if (-0.001 < total < 0.002): 
                vel_log.append(total * 0)
                # print('doing')
            else:
                vel_log.append(total)
            # frequency = int(((total) * 1000000) + 220)
            #frequency = total * 100 + 220

            Alast = accel_log[-1]
            Vlast = vel_log[-1]

            #Athreshold = 0.001
            # if -0.005 < Vlast < 0.01:
            #      Vlast = 0

            # print(str(Alast) + " " + str(Vlast))
            recentv = sum(vel_log[-200:-100]) - sum(vel_log[-100:])
            print("sum " + str(recentv))


            if (Alast == 0 and Vlast == 0):
                print("STATIONARY")
                print(str(Alast) + " " + str(Vlast))
                stationary = True
                movingup = False
                movingdown = False


            elif (Alast > 0 and Vlast > 0): 
                print("MOVING UP 1")
                print(str(Alast) + " " + str(Vlast))
                print(velocity)
                stationary = False
                movingup = True
                movingdown = False

            elif ( Alast == 0 and Vlast > 0): 
                print("MOVING UP 2")
                print(str(Alast) + " " + str(Vlast))
                stationary = False
                movingup = True
                movingdown = False


            elif (Alast < 0) and (Vlast > 0): 
                print("MOVING UP 3")
                print(str(Alast) + " " + str(Vlast))
                stationary = False
                movingup = True
                movingdown = False


            elif (Alast > 0) and (Vlast == 0): 
                print("MOVING UP 4")
                print(str(Alast) + " " + str(Vlast))
                stationary = False
                movingup = True
                movingdown = False

            else: 
                print("MOVING DOWN")
                print(str(Alast) + " " + str(Vlast)) 
                stationary = False
                movingup = False
                movingdown = True
            
            if frequency > 0:
                if sum(acceleration_count) >= 2:
                    x = 10
                    while (frequency > 450) or (frequency < 430):
                        for i in range(1000):       
                            print("RESTART")
                            frequency = frequency
                            output_buffer, phase_offset = generate_block(block_size, phase_offset, frequency, samplerate)
                            player.play(output_buffer)
                        break
                    total = 0 
                    accel_log = []
                    vel_log = []
                    acceleration_count = []
                if movingdown:
                    frequency = frequency - 0.1
                    counter = 0

                if movingup:
                    frequency += 0.1
                    counter = 0

                if stationary:
                    frequency = frequency
                    counter +=1

            # print("this is the count " + str(counter))

            print(recentv)
            
            delay_buffer.pop(0)

                
        
            # write code that automatically turn the whole thing off 



            output_buffer, phase_offset = generate_block(block_size, phase_offset, frequency, samplerate)
            delay_buffer.append(output_buffer)

            volume = 0.25
            echo = volume*(delay_buffer[-sensor_value] + delay_buffer[-int(sensor_value/2)] + delay_buffer[-int(sensor_value/4)] + delay_buffer[-int(sensor_value/8)])
                
            new_output = 0.5*(delay_buffer[-1] + echo)     
            time.sleep(0.005)
            player.play(new_output)

            if GPIO.input(10) == GPIO.HIGH:
                print("Button was pushed!")
                main()

            print(frequency)

main()
