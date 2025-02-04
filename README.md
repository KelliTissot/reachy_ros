# Aplicações de controle para o braço robótico - modelo Reachy 

Este repositório contém aplicações de controle para o braço do robô Reachy, utilizando tecnologias como ROS2, Docker, Gazebo e Python. Abaixo estão as instruções para configurar e executar o ambiente de simulação.

## Tecnologias Utilizadas

**ROS2 Humble**: Framework de software para desenvolvimento de robótica. 

**Docker**: Plataforma para criar, deploy e rodar aplicações em containers.

**Gazebo**: Simulador de robótica para testar algoritmos em ambientes virtuais.

**Python**: Linguagem de programação utilizada para desenvolver os algoritmos de controle.


## Instalação e Execução
Siga os passos abaixo para configurar e executar o ambiente de simulação:

1. **Clonar o repositório:**
```bash 
    git clone git@github.com:KelliTissot/reachy_ros.git
```

2. **Iniciar os containers Docker:**
No terminal, execute o seguinte comando para construir e iniciar os containers:
```bash
    docker compose up --build 
```
3. **Acessar o container em execução:**
Em outro terminal, acesse o container Docker em execução com o seguinte comando:
```bash
    docker exec -it <nome_do_container> bash #Substitua <nome_do_container> pelo nome do container que está em execução.
```
4. **Iniciar o ambiente de simulação:**
Dentro do container, execute o seguinte comando para iniciar o ambiente de simulação no Gazebo e o RVIZ:
```bash
    ros2 launch reachy_bringup reachy.launch.py start_sdk_server:=true gazebo:=true start_rviz:=true
```
5. **Executar o algoritmo de controle:**
OBS: O robô seguirá a trajetória definida no arquivo __init__.py.
```bash
    ros2 run reachy_kdl_kinematics rehabot
```
6. **Executar outra trajetória:**
Para executar uma trajetória diferente, substitua o nome do arquivo Python no trecho abaixo, localizado no arquivo __init__.py:

O robô seguirá a trajetória definida no arquivo __init__.py.
Para executar outra trajetória, é necessário trocar o nome do código.py desejado em: 
```bash
    from rehabot.<nome_do_código_desejado> import main
```
## Estrutura do Projeto

- reachy_bringup/: Contém os arquivos de configuração e lançamento do ROS2.

- reachy_kdl_kinematics/: Contém os algoritmos de cinemática e controle do robô.

- rehabot/: Contém os scripts Python que definem as trajetórias do robô.


## Atribuições

Este projeto utiliza arquivos e recursos do projeto open source **[Pollen Robotics](https://www.pollen-robotics.com/)**, resposnável por desenvolver o robô **Reachy** que serve como base para o projeto RehaBot desde o início. 