'''HARIOMHARIBOLJAIMATAJIPITAJIKIJAIJAI'''

# Script Adaptado para o prototipo de robo pendulo invertido floki

# Importando as bibliotacas utilizadas, lembrando que mpu6050 eh para python2.7
from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import  RPi.GPIO as GPIO



GPIO.setmode(GPIO.BCM) #Setando o modo de pinagem BCM
GPIO.setwarnings(False) 

#Declarando os pinos de controle dos motores
int1 = 21
int2 = 20
int3 = 16
int4 = 12

GPIO.setup(int1, GPIO.OUT)
GPIO.setup(int2, GPIO.OUT)
GPIO.setup(int3, GPIO.OUT)
GPIO.setup(int4, GPIO.OUT)

#Configurando os pinos como PWM
PWM1 = GPIO.PWM(21, 100)
PWM2 = GPIO.PWM(20, 100)
PWM3 = GPIO.PWM(16, 100)
PWM4 = GPIO.PWM(12, 100)


PWM1.start(0)
PWM2.start(0)
PWM3.start(0)
PWM4.start(0)


#Funcao que fara o robo andar pra tras e tem como argumento de entrada o valor de PID
def backward(velocity):
    PWM1.ChangeDutyCycle(velocity)
    GPIO.output(int2, GPIO.LOW)
    PWM3.ChangeDutyCycle(velocity)
    GPIO.output(int4, GPIO.LOW)

#Tal como a funcao anterior, mas fara com que o robo ande pra frente
def forward(velocity):
    GPIO.output(int1, GPIO.LOW)
    PWM2.ChangeDutyCycle(velocity)
    GPIO.output(int3, GPIO.LOW)
    PWM4.ChangeDutyCycle(velocity)

#Funcao para cado o valor do PID seja 0, ou seja, o robo esta na posicao de equilibrio
def equilibrium():
    GPIO.output(int1, False)
    GPIO.output(int2, False)
    GPIO.output(int3, False)
    GPIO.output(int4, False)


sensor = mpu6050(0x68)
#K e K1 --> Constantes para o Filtro Complementar de Shane Colton
K = 0.98
K1 = 1 - K

#define tempo de amostragem
time_diff = 0.02
ITerm = 0

#adiquire os primeiros valores do sensor 
accel_data = sensor.get_accel_data()
gyro_data = sensor.get_gyro_data()

aTempX = accel_data['x']
aTempY = accel_data['y']
aTempZ = accel_data['z']

gTempX = gyro_data['x']
gTempY = gyro_data['y']
gTempZ = gyro_data['z']

#algumas funcoes matematicas 
def distance(a, b):
    return math.sqrt((a*a) + (b*b))

def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)

def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)


last_x = x_rotation(aTempX, aTempY, aTempZ)
last_y = y_rotation(aTempX, aTempY, aTempZ)

gyro_offset_x = gTempX
gyro_offset_y = gTempY

gyro_total_x = (last_x) - gyro_offset_x
gyro_total_y = (last_y) - gyro_offset_y


#o loop principal em que sera feito todo o controle de equilibrio
while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    accelX = accel_data['x']
    accelY = accel_data['y']
    accelZ = accel_data['z']

    gyroX = gyro_data['x']
    gyroY = gyro_data['y']
    gyroZ = gyro_data['z']

    gyroX -= gyro_offset_x
    gyroY -= gyro_offset_y

    gyro_x_delta = (gyroX * time_diff)
    gyro_y_delta = (gyroY * time_diff)

    gyro_total_x += gyro_x_delta
    gyro_total_y += gyro_y_delta

    rotation_x = x_rotation(accelX, accelY, accelZ)
    rotation_y = y_rotation(accelX, accelY, accelZ)
    
    #Filtro COmplementar de Shane Colton
    last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)

    #configuracao dos parametros do controlador PID
    PID = PIDController(P=-78.5, I=1.0, D=1.0)
    PIDy = PID.step(last_y)

    #se PIDy < 0 o sentidos motores sera antihorario com intensidade PIDy
    if PIDy < 0.0:
        if PIDy < -1550:
           PIDy = -1550 
        velocidade = PIDy * (100/1550)
        backward(-float(velocidade))
        #StepperFor(-PIDx)
    #se PIDy > 0 entao o sentido dos motores sera horario com intensidade PIDy
    elif PIDy > 0.0:
        if PIDy > 1550:
            PIDy = 1550
         velocidade = PIDy * (100/1550)
        forward(float(velocidade))
        #StepperBACK(PIDx)
    #se nenhuma das condicoes anteriores for setisfeita, entao o robo esta em equilibrio 
    else:
        equilibrium()


    print(int(last_y), 'PID: ', int(PIDy))
    sleep(0.02)
    
    
