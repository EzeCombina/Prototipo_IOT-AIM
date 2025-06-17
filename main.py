import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import Adafruit_DHT
import time
from multiprocessing import Process, Queue
import signal
import sys

print("Inicio del programa")

# ==== Configuracion de pines y hardware ====
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

password = "1234"
sensor = Adafruit_DHT.DHT11
pin_dht11 = 4  # GPIO (BCM)
pin_led = 8
pin_servo = 12
pin_rele = 10
pin_ir = 36 
pin_ir_beginning = 38

# Pines del motor paso a paso
IN1, IN2, IN3, IN4 = 21, 22, 23, 24

GPIO.setup(pin_led, GPIO.OUT)
GPIO.output(pin_led, GPIO.LOW)	
GPIO.setup(pin_servo, GPIO.OUT)
GPIO.setup(pin_rele, GPIO.OUT)
GPIO.output(pin_rele, GPIO.LOW)	

# MQTT
broker = "192.168.1.93"
port = 1883
topic_temp = "raspberry/temp"
topic_hum = "raspberry/hum"
topic_rele = "raspberry/rele"
topic_switch = "raspberry/switch"
topic_slider = "raspberry/slider"
topic_ir = "raspberry/ir"
topic_reset = "raspberry/reset"

pwm = GPIO.PWM(pin_servo, 50) 							# Frecuencia de 50 Hz
pwm.start(0) 											# Se inicia el PWM con un ciclo de trabajo de 0%				

intervalo 			= 1000								# Tiempo del intervalo en ms
ultimo_tiempo 		= time.perf_counter_ns() / 1000000 	# Tiempo en ms

switch_state = False
angulo = 0

def crear_cliente_mqtt():
    client = mqtt.Client()
    client.username_pw_set(username="ezequiel", password=password)
    client.on_message = on_message   # Asignamos la funcion que maneja los mensajes
    client.connect(broker, port, keepalive=60)
    client.loop_start()              # Empieza el loop en segundo plano
    return client

def sensor_ir_process(queue_flag_contador):
    # Crear un nuevo cliente MQTT para este proceso
    # En Python, los procesos no comparten memoria ni objetos como client
    local_client = mqtt.Client()
    local_client.username_pw_set(username="ezequiel", password=password)
    local_client.connect(broker, port, keepalive=60)
    local_client.loop_start()
	
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin_ir, GPIO.IN)

    estado_anterior = None
    contador = 0
    local_client.publish(topic_ir, f"{contador}")

    try:
        while True:
            estado = GPIO.input(pin_ir)
            
            # Verificar si se debe resetear el contador
            if not queue_flag_contador.empty():
                flag = queue_flag_contador.get()
                if flag == 1:
                    print("Reset del contador")
                    contador = 0
                    local_client.publish(topic_ir, f"{contador}")
                    
            if estado != estado_anterior:
                if estado == 0:
                    contador += 1
                    local_client.publish(topic_ir, f"{contador}")
                    print("Obstaculo detectado")
                else:
                    print("Camino libre")
                estado_anterior = estado
            time.sleep(0.1)
    except KeyboardInterrupt:
        GPIO.cleanup()

def motor_process():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(pin_ir_beginning, GPIO.IN)

    step_sequence = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1],
    ]

    def step_motor(sequence, steps=1, delay_time=0.001):
        for _ in range(steps):
            for step in sequence:
                GPIO.output(IN1, step[0])
                GPIO.output(IN2, step[1])
                GPIO.output(IN3, step[2])
                GPIO.output(IN4, step[3])
                time.sleep(delay_time)

    try:
        print("Giro inicial en sentido antihorario hasta que el sensor IR detecte")
        while GPIO.input(pin_ir_beginning) == 1:
            step_motor(step_sequence[::-1], steps=1)

        print("Sensor IR activado, bajando un poco mas")
        step_motor(step_sequence[::-1], steps=100)  

        print("Comienza ciclo normal")
        while True:
            print("Motor horario")
            step_motor(step_sequence, 512)
            time.sleep(1)
            print("Motor antihorario")
            step_motor(step_sequence[::-1], 512)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup([IN1, IN2, IN3, IN4])

# Funciones
def mover_servo(angulo):
	# Se convierte el Ã¡ngulo a un ciclo de trabajo del 2% al 12%
	duty_cycle = 2 + (angulo / 18)
	pwm.ChangeDutyCycle(duty_cycle)
	time.sleep(0.5) # Tiempo para que llegue a la posecion 
	pwm.ChangeDutyCycle(0)

def on_message(client, userdata, msg):
	global switch_state
	global angulo
	try:
		if msg.topic == topic_switch:
			switch_state = msg.payload.decode() == "True"
			print(f"Estado del switch: {'Encendido' if switch_state else 'Apagado'}")
			GPIO.output(pin_led, GPIO.HIGH if switch_state else GPIO.LOW)
		if msg.topic == topic_slider:
			angulo = int(msg.payload.decode().strip())
			print(f"La nueva posicion del servo es de {angulo}Â°")
			mover_servo(angulo)
		elif msg.topic == topic_reset:
			queue_flag_contador.put(1)  # Poner el flag en la cola
	except Exception as e:
		print(f"Error procesando el mensaje: {e}")

# === MAIN ===
if __name__ == "__main__":
    try:
		# Cliente
        client = crear_cliente_mqtt()
        client.subscribe(topic_switch)
        client.subscribe(topic_slider)
        client.subscribe(topic_ir)
        client.subscribe(topic_reset)
        
        queue_flag_contador = Queue()
		
        procesos = [
            Process(target=motor_process),
            Process(target=sensor_ir_process, args=(queue_flag_contador,))
        ]

        for p in procesos:
            p.start()
        #for p in procesos:
        #    p.join()
        
        while True:
			# Tomamos el valor de la temperatura y de la humedad 
            humedad, temperatura = Adafruit_DHT.read(sensor, pin_dht11)
			
            if humedad is not None and temperatura is not None:
                # Leemos el volar de tiempo actual
                tiempo_actual = time.perf_counter_ns() / 1000000
				
                client.publish(topic_temp, f"{temperatura:.2f}")
                client.publish(topic_hum, f"{humedad:.2f}")
				
                if humedad > 85:
                    GPIO.output(pin_rele, GPIO.HIGH)
                    client.publish(topic_rele, "1")
                else:
                    GPIO.output(pin_rele, GPIO.LOW)
                    client.publish(topic_rele, "0")	
				
                # Printeamos cada un cierto tiempo, en este caso 1 seg
                if tiempo_actual - ultimo_tiempo >= intervalo:
                    print(f"Temperatura: {temperatura:.1f}Â°C, Humedad: {humedad:.1f}%")
                    ultimo_tiempo = tiempo_actual
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Terminando...")
    finally:
        pwm.stop()  # Detiene el PWM correctamente antes de limpiar
        GPIO.cleanup()
        sys.exit(0)