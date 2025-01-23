# Aplicações de controle para o braço do robô REHABOT-FURG

## Tecnologias

- <img src="assets/ros.svg" alt="ROS2" width="40"/> - **ROS2 Humble**
- <img src="assets/docker.svg" alt="Docker" width="40"/> - **Docker**
- <img src="assets/probot.svg" alt="Gazebo" width="40"/> - **Gazebo**
- <img src="assets/python.svg" alt="Python" width="40"/> - **Python**



## Instalação
1. Clonar o repositório:
```bash 
    git clone git@github.com:KelliTissot/reachy_ros.git
```

2. Ambiente Virtual
```bash
    python3 -m venv venv #criar ambiente virtual
    source /venv/bin/activate # ativar ambiente dentro do ubuntu

```

3. Dentro do ambiente virtual
```bash
    docker compose up -d #iniciar conteiners
```
4. Em outro terminal
```bash
    docker exec -it <nome_do_container> bash #abrir um terminal interativo dentro do conteiner em execução
```
5. Abrir o ambiente de simulação
```bash
    ros2 launch reachy_bringup reachy.launch.py start_sdk_server:=true gazebo:=true start_rviz:=true
```
