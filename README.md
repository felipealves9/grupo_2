# Projeto Final Trainee Eletrônica AVANT - Faulty or not

Este repositório contém a solução do projeto final do Trainee de Eletrônica, englobando visão computacional, navegação e controle através do ROS 2. Abaixo está o guia passo a passo para configurar o ambiente Docker, posicionar os arquivos corretamente e executar a simulação.

## 1. Configuração e Execução do Docker

Para garantir o funcionamento adequado da interface gráfica e do servidor de áudio da sua máquina local no container, utilize os comandos abaixo.

### Executando pela primeira vez

No terminal da sua máquina, execute o comando abaixo para baixar a imagem, criar e acessar o container:

    docker run -it \
      --name trainee-26 \
      --network host \
      --group-add audio \
      --group-add dialout \
      -e DISPLAY=$DISPLAY \
      -e QT_X11_NO_MITSHM=1 \
      -e PULSE_SERVER=unix:/tmp/pulse-socket \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v /run/user/$(id -u)/pulse/native:/tmp/pulse-socket \
      -v /dev/snd:/dev/snd \
      joaoaugustoc/trainee-26:main \
      bash

### Para as próximas vezes

Caso o container já tenha sido criado (após você ter fechado o terminal ou desligado o computador), basta iniciá-lo e acessá-lo:

    docker start trainee-26 
    docker exec -it trainee-26 bash

## 2. Instalação das Dependências de Áudio

Como a missão exige a reprodução de arquivos de áudio para cada forma geométrica identificada, precisamos instalar um reprodutor de áudio via terminal no container.

Dentro do container, execute:

    apt update
    apt install mpg123 -y

Nota: Caso não esteja logado como usuário root, adicione `sudo` antes dos comandos.

## 3. Estrutura de Arquivos e Pastas

O ambiente de simulação e o seu workspace (avant_ws) já possuem uma estrutura base. Os arquivos disponibilizados neste repositório (setup.py, takeoff.py, vision.py e os áudios .mp3) devem ser colocados estritamente na seguinte árvore de diretórios:

    ~/simulation/starts/avant_ws/
    ├── build/
    ├── image/
    ├── install/
    ├── log/
    └── src/
        └── pkg_avant/
            ├── setup.py                 <-- Substitua o arquivo setup.py aqui
            ├── package.xml
            └── pkg_avant/
                ├── __init__.py
                ├── takeoff.py           <-- Arquivo de navegação
                ├── vision.py            <-- Arquivo de visão computacional
                ├── som1.mp3             <-- Áudio do Triângulo
                ├── som2.mp3             <-- Áudio do Hexágono
                └── som3.mp3             <-- Áudio do Círculo

Certifique-se de que os scripts Python e os áudios foram movidos para a pasta interna `pkg_avant`, e que o arquivo `setup.py` está na raiz do pacote (um nível acima dos scripts).

## 4. Compilação do Pacote ROS 2

Sempre que adicionar ou alterar os arquivos (especialmente o setup.py e os áudios), é necessário recompilar o workspace para que o ROS 2 copie os arquivos para os diretórios de instalação corretos.

Dentro do container, navegue até a raiz do workspace e compile o pacote:

    cd ~/simulation/starts/avant_ws
    colcon build --packages-select pkg_avant
    source install/setup.bash

## 5. Execução da Simulação

Para executar o projeto completo, voce precisara de tres terminais abertos dentro do container Docker.

### Terminal 1: Iniciar o ambiente de simulação

Abra o primeiro terminal no container e rode o script do tmux já configurado, que inicializa todas as dependências físicas e o ambiente virtual:

    cd ~/simulation/starts
    ./desafio.sh

### Terminal 2: Iniciar o nó de Visão Computacional

Abra uma nova aba/terminal, acesse o container e inicie o nó responsável pela detecção de imagens e reprodução de áudio:

    cd ~/simulation/starts/avant_ws
    source install/setup.bash
    ros2 run pkg_avant mission_vision_node

### Terminal 3: Iniciar o nó de Navegação

Abra uma terceira aba/terminal, acesse o container e inicie o nó responsável pelo controle e movimentação do drone:

    cd ~/simulation/starts/avant_ws
    source install/setup.bash
    ros2 run pkg_avant simple_takeoff_land

Com os três terminais rodando, o drone fará a varredura da arena, identificará as formas geométricas e as cores e, ao final, reproduzirá a sequência de sons correta com base na lógica estabelecida.
