import math
import random
import time
from controller import Robot, Motor, DistanceSensor, Supervisor, Node

TIME_STEP = 32
QtddCaixas = 20
QtddSensores = 8

LIMIAR_MOVIMENTO_CAIXA = 0.015
LIMIAR_PAREDE = 120.0
VELOCIDADE = 6.0
TEMPO_INICIAL_IGNORADO = 50
LIMIAR_TRAVADO = 0.005
CICLOS_TRAVADO = 62

robot = Robot()
supervisor = Supervisor()
random.seed(time.time())

MotorEsquerdo = robot.getDevice("left wheel motor")
MotorDireito = robot.getDevice("right wheel motor")
MotorEsquerdo.setPosition(float('inf'))
MotorDireito.setPosition(float('inf'))
MotorEsquerdo.setVelocity(0)
MotorDireito.setVelocity(0)

sensores = []
for i in range(QtddSensores):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(TIME_STEP)
    sensores.append(sensor)

caixaLeve = None
posicaoInicialX = 0.0
posicaoInicialZ = 0.0
posicaoSalva = False

for i in range(1, QtddCaixas + 1):
    nomeCaixa = f"CAIXA{i:02d}"
    caixa = supervisor.getFromDef(nomeCaixa)
    if caixa is not None:
        campoMassa = caixa.getField("mass")
        if campoMassa:
            massa = campoMassa.getSFFloat()
            if massa == 0.06:
                caixaLeve = caixa
                print(f"Caixa leve encontrada: {nomeCaixa}")
                break

if caixaLeve is None:
    print("Caixa leve não encontrada.")
    robot.cleanup()
    exit(1)

encontrou = False
contador = 0
maxContador = 60 + random.randint(0, 59)
recuando = False
tempoRecuo = 0
ciclos = 0
ciclosTravado = 0
ultimaPosX = 0.0
ultimaPosZ = 0.0

while robot.step(TIME_STEP) != -1:
    ciclos += 1

    posCaixaAtual = caixaLeve.getPosition()
    posRobo = supervisor.getSelf().getPosition()

    if not posicaoSalva and ciclos >= TEMPO_INICIAL_IGNORADO:
        posicaoInicialX = posCaixaAtual[0]
        posicaoInicialZ = posCaixaAtual[2]
        ultimaPosX = posRobo[0]
        ultimaPosZ = posRobo[2]
        posicaoSalva = True
        print(f"Posição inicial da caixa leve salva: x={posicaoInicialX:.5f} z={posicaoInicialZ:.5f}")

    if posicaoSalva and not encontrou:
        dx = posCaixaAtual[0] - posicaoInicialX
        dz = posCaixaAtual[2] - posicaoInicialZ
        deslocamento = math.sqrt(dx**2 + dz**2)
        #print(f"Deslocamento da caixa leve: {deslocamento:.5f}")
        if deslocamento > LIMIAR_MOVIMENTO_CAIXA:
            print("Caixa leve foi empurrada! Iniciando rotação...")
            encontrou = True

    if encontrou:
        MotorEsquerdo.setVelocity(VELOCIDADE)
        MotorDireito.setVelocity(-VELOCIDADE)
        continue

    movX = abs(posRobo[0] - ultimaPosX)
    movZ = abs(posRobo[2] - ultimaPosZ)
    if movX < LIMIAR_TRAVADO and movZ < LIMIAR_TRAVADO:
        ciclosTravado += 1
    else:
        ciclosTravado = 0
        ultimaPosX = posRobo[0]
        ultimaPosZ = posRobo[2]

    if ciclosTravado > CICLOS_TRAVADO:
        print("Robô travado! Recuando...")
        MotorEsquerdo.setVelocity(-VELOCIDADE)
        MotorDireito.setVelocity(-VELOCIDADE)
        for _ in range(10):
            if robot.step(TIME_STEP) == -1:
                break
        MotorEsquerdo.setVelocity(VELOCIDADE)
        MotorDireito.setVelocity(VELOCIDADE * 0.2)
        for _ in range(10):
            if robot.step(TIME_STEP) == -1:
                break
        ciclosTravado = 0
        continue

    ps0 = sensores[0].getValue()
    ps1 = sensores[1].getValue()
    ps6 = sensores[6].getValue()
    ps7 = sensores[7].getValue()

    caixa_proxima = False
    for i in range(1, QtddCaixas + 1):
        nomeCaixa = f"CAIXA{i:02d}"
        caixa = supervisor.getFromDef(nomeCaixa)
        if caixa is not None:
            posCaixa = caixa.getPosition()
            dx = posCaixa[0] - posRobo[0]
            dz = posCaixa[2] - posRobo[2]
            dist = math.sqrt(dx**2 + dz**2)
            if dist < 0.09:
                caixa_proxima = True
                break

    sensores_parede = (1 if ps0 > LIMIAR_PAREDE else 0) + \
                      (1 if ps1 > LIMIAR_PAREDE else 0) + \
                      (1 if ps6 > LIMIAR_PAREDE else 0) + \
                      (1 if ps7 > LIMIAR_PAREDE else 0)
    
    # só considera parede se 2 ou mais sensores detectarem forte e não for caixa
    parede = (sensores_parede >= 2) and not caixa_proxima

    if parede and not recuando:
        print("Parede detectada! Recuando...")
        recuando = True
        tempoRecuo = 15
        MotorEsquerdo.setVelocity(-VELOCIDADE)
        MotorDireito.setVelocity(-VELOCIDADE)
    elif recuando:
        tempoRecuo -= 1
        if tempoRecuo <= 0:
            recuando = False
            if random.randint(0, 1) == 0:
                MotorEsquerdo.setVelocity(VELOCIDADE * 0.3)
                MotorDireito.setVelocity(VELOCIDADE)
            else:
                MotorEsquerdo.setVelocity(VELOCIDADE)
                MotorDireito.setVelocity(VELOCIDADE * 0.3)
    else:
        contador += 1
        if contador >= maxContador:
            dir = random.randint(0, 2)
            if dir == 0:
                MotorEsquerdo.setVelocity(VELOCIDADE)
                MotorDireito.setVelocity(VELOCIDADE)
            elif dir == 1:
                MotorEsquerdo.setVelocity(VELOCIDADE * 0.5)
                MotorDireito.setVelocity(VELOCIDADE)
            else:
                MotorEsquerdo.setVelocity(VELOCIDADE)
                MotorDireito.setVelocity(VELOCIDADE * 0.5)
            contador = 0
            maxContador = 60 + random.randint(0, 79)

robot.cleanup()