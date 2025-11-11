
# Sistema de Monitoramento Vibracional de Motores Elétricos com ESP32 e MPU-6050

Este projeto implementa um sistema de monitoramento vibracional de motores elétricos utilizando um microcontrolador ESP32 e um sensor MPU-6050. A proposta tem como objetivo realizar a coleta e o processamento de dados de vibração, utilizando a Transformada Rápida de Fourier (FFT) e publicando os resultados em tempo real via protocolo MQTT. O sistema é ideal para aplicações de manutenção preditiva em motores de pequeno e médio porte.

## Componentes Necessários

- **ESP32 DevKit**: Microcontrolador utilizado para o processamento e comunicação.
- **MPU-6050**: Sensor de aceleração e giroscópio utilizado para medir as vibrações do motor.
- **Mosquitto**: Broker MQTT para recebimento das publicações.
- **Aplicativo IoTMQTTPanel (iOS/Android)**: Para visualização dos dados via MQTT.

## Funcionalidade do Sistema

O protótipo realiza a coleta de dados de vibração do motor em tempo real, utilizando o sensor MPU-6050. Os dados são processados no microcontrolador ESP32, onde a Transformada Rápida de Fourier (FFT) é aplicada para analisar a frequência das vibrações. As métricas de vibração (como RMS, frequência de pico e amplitude) são publicadas no protocolo MQTT, permitindo que os dados sejam visualizados em tempo real no aplicativo IoTMQTTPanel ou em qualquer outra aplicação compatível com MQTT.

## Configuração do Sistema

### 1. Configuração do Mosquitto (Broker MQTT)

O broker MQTT utilizado foi o **Mosquitto** versão 2.0.22, que foi instalado no Windows 10 a partir do site oficial do Mosquitto ([link do Mosquitto](https://mosquitto.org/download)).

- Após a instalação, edite o arquivo de configuração `mosq.conf` para permitir conexões anônimas e definir a porta de escuta como `1883`.

Conteúdo de exemplo do arquivo `mosq.conf`:

```
allow_anonymous true
listener 1883 0.0.0.0
```

Inicie o broker com o seguinte comando:

```bash
cd "C:\Program Files\mosquitto"
mosquitto -c mosq.conf
```

Para verificar as mensagens publicadas, utilize o seguinte comando no terminal:

```bash
"C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -t "tcc/vibration/#" -v
```

### 2. Configuração do Cliente MQTT (ESP32)

Edite o código fornecido com os detalhes da sua rede Wi-Fi e broker MQTT. A comunicação com o broker é realizada pela biblioteca [PubSubClient](https://github.com/knolleary/pubsubclient), versão 2.8.0, disponível no repositório GitHub.

Defina as credenciais e o tópico MQTT no código:

```cpp
#define WIFI_SSID   "your_wifi_ssid"
#define WIFI_PASS   "your_wifi_password"
#define MQTT_BROKER "your_mqtt_broker_ip"
#define MQTT_PORT   1883
#define MQTT_USER   "your_mqtt_user"
#define MQTT_PASS   "your_mqtt_password"
#define MQTT_CLIENTID "esp32-vibe-01"
```

### 3. Configuração do Aplicativo IoTMQTTPanel (iOS/Android)

- Baixe o aplicativo **IoTMQTTPanel** no seu dispositivo móvel.
- Configure o aplicativo para conectar ao broker MQTT, utilizando o `clientID`, o `broker address` e a `porta`.
- O tópico configurado será `tcc/vibration/summary`.

### 4. Funcionalidade do Sistema

O protótipo coleta dados de vibração do motor em tempo real usando o sensor MPU-6050, realiza o pré-processamento dos sinais, calcula a Transformada Rápida de Fourier (FFT), e publica as métricas relevantes via MQTT para visualização. O sistema pode ser integrado a sistemas de monitoramento em tempo real, como o aplicativo IoTMQTTPanel.

## Como Usar

1. Conecte o ESP32 à sua rede Wi-Fi.
2. Compile e faça o upload do código para o ESP32 usando o Arduino IDE.
3. Inicie o Mosquitto no seu computador.
4. Abra o aplicativo IoTMQTTPanel no seu dispositivo móvel e conecte-se ao broker MQTT.
5. Visualize as métricas publicadas em tempo real no seu painel de controle.

## Explicação do Código

O código implementa a funcionalidade descrita de monitoramento vibracional. Aqui está uma explicação das partes principais:

### Configurações Iniciais

- **WIFI_SSID** e **WIFI_PASS**: Defina o nome e a senha da sua rede Wi-Fi.
- **MQTT_BROKER**: O endereço IP do broker MQTT onde os dados serão enviados.
- **MQTT_CLIENTID**: Identificador único para o cliente MQTT.
- **MQTT_TOPIC_SUMMARY** e **MQTT_TOPIC_DEBUG**: Tópicos onde os dados de vibração serão publicados.

### Funções Principais

1. **Conexão Wi-Fi**: A função `connectWiFi()` conecta o ESP32 à rede Wi-Fi utilizando o SSID e senha fornecidos.

2. **Conexão MQTT**: A função `connectMQTT()` conecta o ESP32 ao broker MQTT, usando o `clientID`, `user`, e `pass`. O ESP32 também publica uma mensagem de status inicial indicando que está online.

3. **Calibração do Sensor**: A função `calibrateBaseline()` realiza a calibração do sensor MPU-6050 para compensar as leituras de aceleração e giroscópio com base nos valores iniciais do sensor.

4. **Coleta e Processamento de Dados**: 
   - **`sampleBlock()`**: Coleta os dados de aceleração do sensor MPU-6050 e os armazena para análise.
   - **`computeRMS()`**: Calcula o valor RMS dos dados coletados.
   - **`computeFFT_andPeak()`**: Realiza a FFT sobre os dados de vibração para extrair a frequência de pico e sua amplitude.

5. **Publicação MQTT**: O payload JSON contém todas as métricas calculadas, incluindo:
   - Média (\( 	ext{mean} \))
   - RMS bruto e dinâmico (\( 	ext{rms\_raw}, 	ext{rms\_dyn} \))
   - Frequência de pico (\( 	ext{peak\_freq} \))
   - Energia angular do giroscópio (\( 	ext{gyro\_rms\_dps} \))
   - Estado de movimento (\( 	ext{moving} \))

Essas informações são publicadas no broker MQTT para posterior visualização no aplicativo IoTMQTTPanel ou outro sistema.

## Bibliotecas Usadas

- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050) (Versão 2.2.6)
- [arduinoFFT](https://github.com/kosme/arduinoFFT) (Versão 2.0.4)
- [PubSubClient](https://github.com/knolleary/pubsubclient) (Versão 2.8)

## Custo do Hardware

O custo do protótipo foi mantido baixo, utilizando componentes de fácil acesso e baixo custo:

- **ESP32 DevKit**: R$ 50,00 - R$ 65,00
- **MPU-6050**: R$ 18,90 - R$ 37,52
- **Custo Total**: Aproximadamente R$ 80 - R$ 130

