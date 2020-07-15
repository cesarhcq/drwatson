# Tutorial Computacional - (GuntherOBT)

## Instalar Dependências 

- [x] Instalação Sistema Operacional Linux: [Ubuntu 16.04 LTX (Xenial)](https://canaltech.com.br/linux/como-instalar-o-ubuntu-1604-xenial-xerus-em-seu-computador/).
- [x] ROS Kinetic-devel: [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [x] Rosserial: [Pacote para o arduino - Robô Físico](http://wiki.ros.org/rosserial).

 Em seguida, será necessário baixar todos os arquivos de software, drive e de programação do GuntherBOT.

## Clonar e Compilar o repositório manualmente

1. Crie um espaço de trabalho ROS simples caso ainda não o possui. 

```
cd

mkdir -p ~/guntherBot_ws/src && cd ~/guntherBot_ws

catkin init

cd ~/guntherBot_ws/src/ 

git clone git@github.com:ActaVisio/GuntherBot.git
```

2. Realize a compilação do repositório.

```
cd ~/guntherBot_ws/

catkin_make

```

## Carregar Variáveis de Ambiente ROS

Deve-se carregar as variáveis de ambiente a cada novo terminal(shell) criado para execução da simulação. 

```
cd ~/guntherBot_ws/

source devel/setup.bash
```

É possível adicionar automaticamente as variáveis de ambiente do ROS à sua sessão do bash sempre que um novo terminal(shell) for iniciado:

```
echo "source ~/guntherBot_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

## Clonar Repositório automáticamente, instalação via (arquivo .sh)

Realiza o download do arquivo `install-guntherbot.sh` e execute, lembre-se de liberar a permissão através do comando `chmod`.

```
chmod 777 install-guntherbot.sh

./install-guntherbot.sh
```