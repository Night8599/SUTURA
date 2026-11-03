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
                    pyniryo 1.1.2 ── TCP :40001 ──► Ned1
```

| Arquivo | Papel |
|---|---|
| `sutura/drivers/ned1_driver.py` | Implementação de `BaseArmDriver` para o Ned1 |
| `sutura/drivers/joint_limits.py` | Limites das juntas do Ned1 e validação dos comandos |
| `sutura/drivers/arm_driver_node.py` | Nó ROS 2: tópicos, serviços, thread de execução |
| `sutura/drivers/ned1_smoke_test.py` | Teste no hardware sem ROS |
| `test/test_ned1_driver.py` | Testes unitários com um robô simulado |

## Restrições do hardware (importante)

1. **Biblioteca:** o Ned1 roda o software do robô ≤ v4.x, suportado apenas pelo `pyniryo ≤ 1.1.2`. A partir do 1.2.0 só Ned2/Ned3Pro são suportados. O `pyniryo2` foi descontinuado pela Niryo em junho de 2025.
2. **Movimentos bloqueantes:** `move_joints` só retorna ao fim do movimento. O nó executa os comandos numa thread separada para não travar o executor do ROS.
3. **Um único cliente TCP:** durante um movimento o robô não pode ser consultado; `/joint_states` repete a última posição conhecida até o movimento terminar.
4. **Sem parada por software:** a API TCP não tem comando de "stop". O serviço `/sutura/arm/emergency_stop` bloqueia novos comandos, mas **não interrompe um movimento em curso**. O modo de aprendizado (sem torque) não é usado como parada porque o braço cairia. **Durante os testes, mantenha o interruptor de energia ao alcance da mão.**
5. **Sem leitura de torque:** as juntas 1–3 são motores de passo da Niryo e não fornecem torque. `get_joint_torques` retorna `NaN`. A detecção de força no pad terá de vir do iSense ou de outro sensor.
6. **Margem perto do repouso:** a pose de repouso do Ned1 (`joint_3 = -1,30 rad`) fica a apenas 0,04 rad do limite físico (-1,34 rad). Por isso a margem padrão é 0,02 rad; uma margem maior bloqueia qualquer movimento a partir do repouso.

## Segurança embutida no driver

- Comandos fora dos limites são **rejeitados**, nunca saturados.
- `max_step_rad` (padrão 0,5 rad) rejeita deslocamentos grandes num único comando.
- Velocidade limitada a `velocity_percent` (padrão 20 %).
- Valores `NaN`/infinito e vetores com tamanho errado são rejeitados.

## Instalação

```bash
pip install "pyniryo==1.1.2"
cd ~/sutura_ws && colcon build --packages-select sutura && source install/setup.bash
```

Rede: no modo hotspot o robô fica em `10.10.10.10`; via cabo Ethernet direto, em `169.254.200.200`.

## Procedimento para o primeiro movimento real

1. Braço em área livre, sem ferramenta, interruptor de energia ao alcance.
2. Teste sem ROS (move uma junta 0,1 rad e volta, pedindo confirmação a cada passo):
   ```bash
   ros2 run sutura ned1_smoke_test --ip 10.10.10.10 --joint 1 --delta 0.1 --velocity 10
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
| `arm.ned1.ip_address` | `10.10.10.10` | IP do robô |
| `arm.ned1.velocity_percent` | 20 | Velocidade máxima (1–100 %) |
| `arm.ned1.auto_calibrate` | true | Calibra na conexão se necessário (o braço se move) |
| `arm.ned1.safety_margin_rad` | 0.02 | Margem dos limites físicos |
| `arm.ned1.max_step_rad` | 0.5 | Deslocamento máximo por comando (≤ 0 desativa) |

## Próximos passos sugeridos

- Verificar os limites de `joint_limits.py` com o firmware real do robô (vêm do URDF `ned` atual da Niryo).
- Decidir a fonte de medição de força, já que o Ned1 não fornece torque.
- Avaliar, para movimentos finos de sutura, o envio de trajetórias completas (`execute_trajectory_from_poses_and_joints`) em vez de pontos isolados, evitando paradas entre waypoints.
