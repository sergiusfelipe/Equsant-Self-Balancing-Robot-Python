'''HARIOMHARIBOLJAIMATAJIPITAJIKIJAIJAI'''

# Algoritmo para equilibrio do floki

# Importando todas as bibliotedas e classes necessarias
from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import  RPi.GPIO as GPIO



GPIO.setmode(GPIO.BCM) # Utilizando a pinagem no modo BCM 
GPIO.setwarnings(False) # Desabilitando avisos

# Declarnado os pino GPIO que serao utilizados
int1 = 21
int2 = 20
int3 = 16
int4 = 12

GPIO.setup(int1, GPIO.OUT)
GPIO.setup(int2, GPIO.OUT)
GPIO.setup(int3, GPIO.OUT)
GPIO.setup(int4, GPIO.OUT)

# Pulse width modulation: a velocidade muda de acordo com o angulo
PWM1 = GPIO.PWM(21, 100)
PWM2 = GPIO.PWM(20, 100)
PWM3 = GPIO.PWM(16, 100)
PWM4 = GPIO.PWM(12, 100)

# Da inicio aos PWM
PWM1.start(0)
PWM2.start(0)
PWM3.start(0)
PWM4.start(0)


# Funcao que tem como argumento de entrada o valor obtido do PID, movendo os motores de ré
def backward(velocity):
    PWM1.ChangeDutyCycle(velocity)
    GPIO.output(int2, GPIO.LOW)
    PWM3.ChangeDutyCycle(velocity)
    GPIO.output(int4, GPIO.LOW)

# Assim como na funcao anterior, a entrada se dá pelo valor do PID só que movendo os motores no sentido contrário
def forward(velocity):
    GPIO.output(int1, GPIO.LOW)
    PWM2.ChangeDutyCycle(velocity)
    GPIO.output(int3, GPIO.LOW)
    PWM4.ChangeDutyCycle(velocity)

# Para caso o valor do PID for 0, ou seja, o robo está em equilibrio
def equilibrium():
    GPIO.output(int1, False)
    GPIO.output(int2, False)
    GPIO.output(int3, False)
    GPIO.output(int4, False)


sensor = mpu6050(0x68)
# K e K1 --> COnstantes para o Filtro Complementar de Shane Colton
K = 0.98
K1 = 1 - K

time_diff = 0.02
ITerm = 0

# Requisita os dados do MPU6050 
accel_data = sensor.get_accel_data()
gyro_data = sensor.get_gyro_data()

aTempX = accel_data['x']
aTempY = accel_data['y']
aTempZ = accel_data['z']

gTempX = gyro_data['x']
gTempY = gyro_data['y']
gTempZ = gyro_data['z']

# Funcoes basicas de matematica que serao utilizados 
def distance(a, b):
    return math.sqrt((a*a) + (b*b))

def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)

def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)

# Seta as posicoes iniciais do sensor
last_x = x_rotation(aTempX, aTempY, aTempZ)
last_y = y_rotation(aTempX, aTempY, aTempZ)

gyro_offset_x = gTempX
gyro_offset_y = gTempY

gyro_total_x = (last_x) - gyro_offset_x
gyro_total_y = (last_y) - gyro_offset_y


# O loop principal do algoritmo onde sera feita todo o o controle de equilibrio do robo
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
    
    # Filtro Complementar de Shane Colton
    last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)

    # Inicializando o controlador PID juntamente com suas constantes
    PID = PIDController(P=-78.5, I=1.0, D=1.0)
    PIDx = PID.step(last_x)

    # Se PIDx < 0 entao o sentido dos motores será de ré
    if PIDx < 0.0:
        backward(-float(PIDx))
        #StepperFor(-PIDx)
    # Se PIDx > entao o sentido dos motores será frente
    elif PIDx > 0.0:
        forward(float(PIDx))
        #StepperBACK(PIDx)
    # E no caso de PIDx = 0 então o robo está em equilíbrio 
    else:
        equilibrium()


    print(int(last_x), 'PID: ', int(PIDx))
    sleep(0.02)
