# Driver do braço Niryo Ned1

Este documento descreve o driver `Ned1ArmDriver` e o procedimento para os primeiros movimentos do braço no mundo real.

## Arquitetura

```
planning / control ──► /joint_commands (Float32MultiArray, rad)
                              │
                     arm_driver_node  ──► /joint_states (JointState)
                              │       ──► /sutura/hardware_status (String)
                     fila de comandos + thread de execução
                              │
                       Ned1ArmDriver (BaseArmDriver)
                              │  validação: limites, margem, passo máximo
                              │
                    pyniryo 1.0.5 ── TCP :40001 ──► Ned1 (software 3.1.1)
```

| Arquivo | Papel |
|---|---|
| `sutura/drivers/ned1_driver.py` | Implementação de `BaseArmDriver` para o Ned1 |
| `sutura/drivers/joint_limits.py` | Limites das juntas do Ned1 e validação dos comandos |
| `sutura/drivers/arm_driver_node.py` | Nó ROS 2: tópicos, serviços, thread de execução |
| `sutura/drivers/ned1_smoke_test.py` | Teste no hardware sem ROS |
| `test/test_ned1_driver.py` | Testes unitários com um robô simulado |
| `test/test_ned1_pyniryo_client.py` | Testes com o cliente real do pyniryo 1.0.5 contra um servidor TCP falso (pulados se o pyniryo não estiver instalado) |

## Restrições do hardware (importante)

1. **Biblioteca:** o Ned1 da equipe roda o software 3.1.1. Pela tabela de compatibilidade da Niryo (`docs/versions/compatibility.rst` do pyniryo), a versão compatível é o `pyniryo 1.0.5` (sistemas ≤ 3.2.0). O 1.1.2 exige software ≥ 4.1.1 e, a partir do 1.2.0, só Ned2/Ned3Pro são suportados. O `pyniryo2` foi descontinuado pela Niryo em junho de 2025.
2. **Movimentos bloqueantes:** `move_joints` só retorna ao fim do movimento. O nó executa os comandos numa thread separada para não travar o executor do ROS.
3. **Um único cliente TCP:** durante um movimento o robô não pode ser consultado; `/joint_states` repete a última posição conhecida até o movimento terminar.
4. **Sem parada por software:** a API TCP não tem comando de "stop". O serviço `/sutura/arm/emergency_stop` bloqueia novos comandos, mas **não interrompe um movimento em curso**. O modo de aprendizado (sem torque) não é usado como parada porque o braço cairia. **Durante os testes, mantenha o interruptor de energia ao alcance da mão.**
5. **Sem leitura de torque:** as juntas 1–3 são motores de passo da Niryo e não fornecem torque. `get_joint_torques` retorna `NaN`. A detecção de força no pad terá de vir do iSense ou de outro sensor.
6. **Margem perto do repouso:** a pose de repouso do Ned1 (`joint_3 = -1,30 rad`) fica a apenas 0,04 rad do limite físico (-1,34 rad). Por isso a margem padrão é 0,02 rad; uma margem maior bloqueia qualquer movimento a partir do repouso.
7. **Queda do link:** se o pyniryo sinalizar `HostNotReachableException`, `ClientNotConnectedException` ou `InvalidAnswerException` (ou um `OSError`), o driver passa a `is_connected = False`, o nó publica `DISCONNECTED` e para de publicar `/joint_states`, para que posições antigas não passem por atuais. Não há reconexão automática: reinicie o nó. Uma recusa do robô (`NiryoRobotException`, "Command KO") mantém a conexão.
8. **Sem timeout depois de conectar:** o pyniryo 1.0.5 usa timeout de 5 s só no `connect`; depois o socket fica sem timeout. Se o robô sumir sem fechar a conexão (cabo desconectado, queda de energia), uma chamada em andamento pode bloquear sem limite, e `disconnect()` e o encerramento do nó esperam por ela. Nos testes reais, deixe o interruptor ao alcance e, se o nó travar, encerre o processo. CONFIRMAR no DRIVER-02.

## Segurança embutida no driver

- Comandos fora dos limites são **rejeitados**, nunca saturados.
- `max_step_rad` (padrão 0,5 rad) rejeita deslocamentos grandes num único comando.
- Velocidade limitada a `velocity_percent` (padrão 20 %).
- Valores `NaN`/infinito e vetores com tamanho errado são rejeitados.
- `/sutura/arm/home` vai para a pose fixa `[0, 0.3, -1.3, 0, 0, 0]` (a pose home do pyniryo 1.0.5) e não passa por `max_step_rad`: de uma pose distante o movimento pode ser grande.

## Instalação

O pyniryo 1.0.5 não entra no `install_requires` do `setup.py`. O `requirements.txt` dele fixa `enum34` (sobrescreve o módulo `enum` da biblioteca padrão do Python 3 e quebra o interpretador) e `opencv-python==4.2.0.32` (versão antiga, sem garantia de pacote para o Python 3.12 do Jazzy). Por isso a instalação é em duas etapas, no mesmo Python do ROS 2 (CONFIRMAR):

```bash
pip install --no-deps "pyniryo==1.0.5"
pip install numpy opencv-python
cd ~/sutura_ws && colcon build --packages-select sutura && source install/setup.bash
```

Conferir que o `enum` continua sendo o da biblioteca padrão:

```bash
python3 -c "import enum, pyniryo; from pyniryo import NiryoRobot; print(enum.__file__)"
```

O caminho impresso deve ser o da biblioteca padrão do Python 3.12 (por exemplo `/usr/lib/python3.12/enum.py`), nunca um arquivo em `site-packages`.

Rede: o padrão do driver é o cabo Ethernet direto, em `169.254.200.200` (a conexão que a equipe usa). No modo hotspot o robô fica em `10.10.10.10` e o PC fica sem internet: instale tudo antes de conectar.

## Testes

```bash
colcon test --packages-select sutura && colcon test-result --verbose
# só os testes do driver, sem ROS:
python3 -m pytest src/sutura/test/test_ned1_driver.py src/sutura/test/test_ned1_pyniryo_client.py
```

`test_ned1_pyniryo_client.py` usa o cliente real do pyniryo contra um servidor TCP falso em `127.0.0.1` e é pulado se o pyniryo não estiver instalado. Nenhum teste acessa o robô.

## Procedimento para o primeiro movimento real

1. Braço em área livre, sem ferramenta, interruptor de energia ao alcance. O robô da equipe estava com `calibration_needed: true`: a conexão calibra sozinha (`arm.ned1.auto_calibrate: true`) e o braço se move durante a calibração.
2. Teste sem ROS (move uma junta 0,1 rad e volta, pedindo confirmação a cada passo):
   ```bash
   ros2 run sutura ned1_smoke_test --joint 1 --delta 0.1 --velocity 10
   ```
   O erro final de cada junta deve ficar abaixo de ~0,01 rad.
3. Teste com ROS, ajustando `arm.type: "ned1"` em `config/sutura_params.yaml`:
   ```bash
   ros2 run sutura arm_driver_node --ros-args --params-file install/sutura/share/sutura/config/sutura_params.yaml
   ros2 topic echo /sutura/hardware_status
   ros2 topic pub --once /joint_commands std_msgs/msg/Float32MultiArray "{data: [0.1, 0.3, -1.3, 0.0, 0.0, 0.0]}"
   ros2 service call /sutura/arm/home std_srvs/srv/Trigger
   ros2 service call /sutura/arm/emergency_stop std_srvs/srv/Trigger
   ros2 service call /sutura/arm/reset_emergency_stop std_srvs/srv/Trigger
   ```

## Parâmetros (`arm.*`)

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `arm.type` | `mock` | `ned1`, `mock` ou `makiina` |
| `arm.state_rate_hz` | 10.0 | Frequência de `/joint_states` |
| `arm.command_queue_size` | 1 | Comandos recebidos com a fila cheia são descartados |
| `arm.ned1.ip_address` | `169.254.200.200` | IP do robô (cabo Ethernet; hotspot: `10.10.10.10`) |
| `arm.ned1.velocity_percent` | 20 | Velocidade máxima (1–100 %) |
| `arm.ned1.auto_calibrate` | true | Calibra na conexão se necessário (o braço se move) |
| `arm.ned1.safety_margin_rad` | 0.02 | Margem dos limites físicos |
| `arm.ned1.max_step_rad` | 0.5 | Deslocamento máximo por comando (≤ 0 desativa) |

## Monitoramento com ned-ros2-driver (opcional, só leitura)

O [ned-ros2-driver](https://github.com/NiryoRobotics/ned-ros2-driver) (ponte ROS 2 Jazzy ↔ ROSBridge, porta 9090 do robô) serve apenas para observar o robô. O movimento do Ned1 é feito só pelo `Ned1ArmDriver`, via pyniryo (decisão D-07). Não use a ponte para mover o braço.

Comando validado no Ned1 (software 3.1.1), com whitelist e namespace próprio:

```bash
ros2 run niryo_ned_ros2_driver ros2_driver --ros-args \
  -p robot_ip:=169.254.200.200 \
  -p robot_namespace:=ned1_monitor \
  -p "topic_whitelist:=['/joint_states', '/niryo_robot_hardware_interface/.*']" \
  -p "service_whitelist:=['/none']" \
  -p "action_whitelist:=['/none']"
```

- **A whitelist é obrigatória.** Sem ela a ponte conecta, lista as interfaces e cai no Ned1 3.1.1 com `No module named 'niryo_robot_commander'`, porque só converte interfaces de pacotes ROS 1 que conhece.
- **Namespace:** a ponte publica `/joint_states`, o mesmo nome do `arm_driver_node`. Com `robot_namespace:=ned1_monitor` os tópicos ganham o prefixo (`/ned1_monitor/joint_states`, pelo código da ponte, `full_topic_name = prefixo + nome`), sem conflito. CONFIRMAR: o comando sem `robot_namespace` foi validado no robô; o prefixo foi derivado do código e não foi testado.
- Não passe valor vazio (`-p robot_namespace:=` é rejeitado pelo rcl).
- `effort` chega 0.0 em todas as juntas (as juntas 1 a 3 não medem torque). O `arm_driver_node` publica NaN.
- O relógio do Ned1 está desatualizado (carimbos em 2021): não use os timestamps do robô.

## Próximos passos sugeridos

- Verificar os limites de `joint_limits.py` com o firmware real do robô (vêm do URDF `ned` atual da Niryo).
- Decidir a fonte de medição de força, já que o Ned1 não fornece torque.
- Avaliar, para movimentos finos de sutura, o envio de trajetórias completas (`execute_trajectory_from_poses_and_joints`) em vez de pontos isolados, evitando paradas entre waypoints.
