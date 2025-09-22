# Exemplo de Publicador e Assinante em ROS2 com Python 🤖

Este repositório contém um exemplo simples de comunicação em ROS2, implementando um sistema com dois nós em Python:

1.  **`talker` (O Falante):** Um nó que publica uma mensagem de texto simples para um tópico a cada segundo. A mensagem contém um contador que incrementa a cada envio (ex: "Olá, mundo! O contador é: 1").
2.  **`listener` (O Ouvinte):** Um nó que se inscreve no mesmo tópico para receber as mensagens enviadas pelo `talker` e as imprime no console assim que chegam.


## Pré-requisitos

* Ubuntu 22.04 (ou compatível)
* ROS2 Humble Hawksbill (ou uma versão mais recente)
* Ferramentas de construção `colcon` e `ament`

## Estrutura do Projeto

O projeto está organizado como um pacote ROS2 `ament_python`.

```
ros2_ws/
└── src/
    └── py_pubsub/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── py_pubsub/
            ├── __init__.py
            ├── talker.py     # Nó publicador
            └── listener.py   # Nó assinante
```

## Guia de Instalação e Execução

Siga os passos abaixo para construir e executar os nós.

### 1. Crie um Workspace ROS2

Se você ainda não tiver um workspace (`ros2_ws`), crie um:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone o Repositório

Dentro da pasta `src`, clone este repositório:

```bash
git clone <https://github.com/diegodemiranda/edrom-ros2-chatter_package> py_pubsub
```

### 3. Construa o Pacote

Volte para a raiz do seu workspace e use `colcon` para construir o pacote:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

### 4. Configure o Ambiente

Após a construção, você precisa "fontar" o ambiente para que o ROS2 encontre os executáveis do seu novo pacote. Faça isso a partir da raiz do workspace:

```bash
source install/setup.bash
```

**Dica:** É uma boa prática adicionar este comando ao seu arquivo `~/.bashrc` para que ele seja executado automaticamente a cada novo terminal.

### 5. Execute os Nós

Agora, você está pronto para executar o sistema. Você precisará de dois terminais separados.

* **Terminal 1: Execute o `talker`**

    Abra um novo terminal, fonte o ambiente e execute o nó `talker`:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run py_pubsub talker
    ```

    A saída esperada será:
    ```
    [INFO] [talker]: Publicando: "Olá, mundo! O contador é: 0"
    [INFO] [talker]: Publicando: "Olá, mundo! O contador é: 1"
    [INFO] [talker]: Publicando: "Olá, mundo! O contador é: 2"
    ...
    ```

* **Terminal 2: Execute o `listener`**

    Abra um segundo terminal, fonte o ambiente e execute o nó `listener`:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run py_pubsub listener
    ```

    A saída esperada será a mensagem recebida do `talker`:
    ```
    [INFO] [listener]: Recebi: "Olá, mundo! O contador é: 0"
    [INFO] [listener]: Recebi: "Olá, mundo! O contador é: 1"
    [INFO] [listener]: Recebi: "Olá, mundo! O contador é: 2"
    ...
    ```

Para parar os nós, simplesmente pressione `Ctrl+C` em cada terminal.
