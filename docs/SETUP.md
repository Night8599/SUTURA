# SUTURA - Guia de ambiente (Ubuntu 24.04 e Windows, ROS 2 Jazzy)

Este guia cobre a instalação do ROS 2 Jazzy nas duas máquinas, o PyNiryo 1.0.5, a rede com o
braço Niryo Ned 1 e os comandos para rodar os testes do pacote `sutura`. Cada passo cita a
fonte oficial usada. Onde a documentação oficial estava bloqueada por proteção anti-bot no
momento da redação (nov/2026), o conteúdo foi obtido do repositório-fonte da documentação
(`ros2/ros2_documentation`, branch `jazzy`, no GitHub) ou de uma versão em cache; os links
abaixo apontam para a página oficial mesmo assim, para conferência.

---

## 1. Ubuntu 24.04 (Noble)

Fonte oficial: [docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 1.1 Locale (UTF-8)

```bash
locale  # checar o locale atual
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # confirmar
```

### 1.2 Repositório APT do ROS 2

Método atual (pacote `ros2-apt-source`, substitui o antigo `apt-key`/gpg manual). Fonte:
[ROS signing key migration guide](https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937)
e [ros2_documentation/_Apt-Repositories.rst](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Se a máquina já tiver uma instalação antiga do ROS 2 feita pelo método manual (gpg + `ros2.list`),
remova `/etc/apt/sources.list.d/ros2.list` e `/usr/share/keyrings/ros-archive-keyring.gpg` antes do
`dpkg -i` acima, para não haver conflito de chaves.

### 1.3 Instalação do ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-dev-tools          # inclui colcon e rosdep
sudo apt install ros-jazzy-desktop      # recomendado: ROS, RViz, demos
# ros-jazzy-ros-base é a opção mínima (sem GUI), não usar aqui (precisamos do RViz)
```

### 1.4 Source do ambiente

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 1.5 rosdep

```bash
sudo rosdep init
rosdep update
```

### 1.6 Workspace e build

Fonte: [Colcon Tutorial (jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

```bash
mkdir -p ~/sutura_ws/src
cd ~/sutura_ws/src
git clone https://github.com/henrique-kado/SUTURA.git
cd ~/sutura_ws
rosdep install --from-paths src --ignore-src -y   # instala dependências de package.xml
colcon build --symlink-install
source install/setup.bash
echo "source ~/sutura_ws/install/setup.bash" >> ~/.bashrc
```

O repositório tem a pasta `src/sutura` (e, depois da INT-3, `src/sutura_interfaces`) dentro de
`src/`; o `colcon build` deve rodar na raiz do workspace (`~/sutura_ws`), não dentro do
repositório clonado.

---

## 2. Windows

Fonte oficial: [docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html).
**Atenção (CONFIRMAR por Felipe no momento da instalação):** essa página ainda documenta o
método antigo via Chocolatey com Python 3.8.3, mas os binários publicados de Jazzy (inclusive
patches recentes) são compilados contra Python 3.12, o que causa erro de caminho/DLL se Python
3.8 for usado (relatado em
[ros2/ros2#1675](https://github.com/ros2/ros2/issues/1675)). O projeto ROS 2 está migrando a
instalação Windows para um método baseado em `pixi` (conda-forge), ver
[Upcoming switch of Windows installation to pixi/conda](https://discourse.openrobotics.org/t/upcoming-switch-of-windows-installation-to-pixi-conda/41916).
Por isso, recomendamos o método pixi abaixo (mais novo e mais estável) em vez do Chocolatey.
Antes de instalar, Felipe deve abrir a página oficial e confirmar qual método ela recomenda
nesse momento; se for o Chocolatey com Python 3.8, seguir a versão de Python que a página
pedir, não a 3.12.

### 2.1 Método recomendado: pixi

1. Criar uma pasta curta para evitar o limite de caminho do Windows, ex.: `C:\ros2_ws`.
2. Instalar o `pixi` (gerenciador de pacotes conda-forge) seguindo
   [prefix-dev.github.io/pixi](https://prefix-dev.github.io/pixi/) (instalador oficial do pixi,
   fora do escopo da doc do ROS).
3. Baixar o pacote binário do Jazzy para Windows em
   [github.com/ros2/ros2/releases](https://github.com/ros2/ros2/releases)
   (arquivo `ros2-jazzy-*-windows-release-amd64.zip`) e extrair em `C:\ros2_ws\ros2-windows`.
4. Em `C:\ros2_ws`, abrir um shell e rodar:

```powershell
pixi shell
call C:\ros2_ws\ros2-windows\local_setup.bat
```

5. Testar com `ros2 run demo_nodes_cpp talker` e, em outro terminal (repetindo os dois comandos
   acima), `ros2 run demo_nodes_py listener`.

### 2.2 Build do workspace (colcon)

```powershell
pip install -U colcon-common-extensions
mkdir C:\ros2_ws\src
cd C:\ros2_ws\src
git clone https://github.com/henrique-kado/SUTURA.git
cd C:\ros2_ws
colcon build --symlink-install
call install\setup.bat
```

### 2.3 Coexistência com Python 3.14

O PC de Felipe já tem Python 3.14 instalado separadamente (uso geral, fora do ROS). O ROS 2
Windows não usa o Python do `PATH` do sistema: no método pixi, o `pixi shell` cria um ambiente
isolado com o Python que o ROS 2 Jazzy espera (hoje, 3.12, conforme a nota do item 2 acima); no
método Chocolatey antigo, o instalador cria `C:\PythonXX` fixo e as ferramentas do ROS apontam
para esse caminho. Em ambos os casos:

- Não desinstalar nem alterar o Python 3.14 do sistema; os dois coexistem sem conflito porque o
  ROS 2 nunca aponta para o Python do `PATH` geral.
- Rodar os comandos do ROS 2 (`ros2 ...`, `colcon ...`, `pip install pyniryo...`) sempre dentro
  do `pixi shell` (ou do prompt com `local_setup.bat`/`setup.bat` carregado), nunca num
  terminal comum que só tenha o Python 3.14.
- Verificar qual Python está ativo com `python --version` dentro desse shell antes de instalar
  qualquer pacote.

---

## 3. PyNiryo 1.0.5 (Ubuntu e Windows)

Mesmo Python do ROS 2 (dentro do ambiente/venv onde o `colcon`/`ros2` rodam), nas duas máquinas:

```bash
python -m pip install pyniryo==1.0.5 --no-deps
python -m pip install numpy
```

Não instalar `enum34` (incompatível com Python 3, causa erro em `enum` da biblioteca padrão).
Confirmar com `python -c "import pyniryo; print(pyniryo.__version__)"`.

---

## 4. Rede com o Ned 1

IP do robô: `169.254.200.200` (porta TCP 40001, PyNiryo). PC: `169.254.200.10/16`, sem gateway
(ver `claude/plano-mestre.md` / `exec-driver.md`).

### 4.1 Ubuntu (nmcli)

Fonte: `man nmcli` / [NetworkManager nmcli reference](https://networkmanager.dev/docs/api/latest/nmcli.html).

```bash
nmcli connection show                       # identificar o nome da conexão cabeada
nmcli con mod "<nome-da-conexao>" \
    ipv4.method manual \
    ipv4.addresses 169.254.200.10/16
nmcli con up "<nome-da-conexao>"
nmcli device status                         # confirmar que a interface está "connected"
ip addr show <interface>                     # confirmar o IP aplicado
```

Não definir `ipv4.gateway` (rede sem gateway, só o robô e o PC).

### 4.2 Windows (PowerShell, como Administrador)

Fonte: [New-NetIPAddress (Microsoft Learn)](https://learn.microsoft.com/en-us/powershell/module/nettcpip/new-netipaddress).

```powershell
Get-NetAdapter                                             # identificar a interface cabeada
New-NetIPAddress -InterfaceAlias "<nome-da-interface>" `
    -IPAddress 169.254.200.10 -PrefixLength 16
```

Não usar `-DefaultGateway` (rede sem gateway). Para remover depois: `Remove-NetIPAddress`.

### 4.3 Teste de conexão (ambas)

```bash
ping 169.254.200.200
ssh niryo@169.254.200.200 -o ServerAliveInterval=15
```

O link Ethernet com o Ned 1 oscila (ver `exec-driver.md`): se o ping ou o SSH falharem na
primeira tentativa, tentar de novo antes de assumir problema de cabeamento.

---

## 5. Rodando os testes

Dentro do workspace (`~/sutura_ws` no Ubuntu, `C:\ros2_ws` no Windows), com o ambiente do ROS 2
e do workspace já "sourced" (passos 1.4/1.6 ou 2.1/2.2):

```bash
colcon test
colcon test-result --verbose      # ver o detalhe das falhas, se houver
```

Os testes marcados `@pytest.mark.hardware` são pulados por padrão (exigem o Ned 1 ligado e na
rede). Para rodá-los com o robô conectado (depois do passo 4.3):

```bash
colcon test --pytest-args -m hardware
# ou, pacote a pacote, direto com pytest:
pytest -m hardware src/sutura/test
```

---

## 6. Resumo rápido (checklist)

- [ ] Locale UTF-8 configurado (Ubuntu).
- [ ] Repositório APT do ROS 2 via `ros2-apt-source` (Ubuntu).
- [ ] `ros-jazzy-desktop` + `ros-dev-tools` instalados (Ubuntu) / binário Jazzy + pixi
      instalados (Windows) - **CONFIRMAR versão de Python exigida na doc oficial no momento da
      instalação**.
- [ ] `rosdep init` + `rosdep update` (Ubuntu).
- [ ] Workspace `src/` com o repositório clonado, `colcon build --symlink-install` ok nas duas
      máquinas.
- [ ] `pyniryo==1.0.5` (`--no-deps`) + `numpy` instalados no mesmo Python do ROS 2, sem
      `enum34`.
- [ ] IP `169.254.200.10/16` sem gateway configurado na interface cabeada (nmcli / New-NetIPAddress).
- [ ] `ping` e `ssh niryo@169.254.200.200` funcionando.
- [ ] `colcon test` roda sem os testes de hardware; `pytest -m hardware` roda com o Ned 1 ligado.
