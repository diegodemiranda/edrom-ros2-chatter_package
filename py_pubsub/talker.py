import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """
    Um nó ROS2 que publica uma mensagem de string em um tópico a cada segundo.
    """
    def __init__(self):
        super().__init__('talker')
        # Cria um publicador para o tópico 'meu_topico' com o tipo de mensagem String
        self.publisher_ = self.create_publisher(String, 'meu_topico', 10)
        
        # Define a frequência de publicação (1 vez por segundo)
        timer_period = 1.0  # segundos
        
        # Cria um temporizador que chama a função 'timer_callback' periodicamente
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Inicializa o contador de mensagens
        self.counter = 0

    def timer_callback(self):
        """
        Função chamada pelo temporizador para publicar a mensagem.
        """
        # Cria uma nova mensagem do tipo String
        msg = String()
        
        # Define o conteúdo da mensagem com o contador
        msg.data = f'Olá, mundo! O contador é: {self.counter}'
        
        # Publica a mensagem no tópico
        self.publisher_.publish(msg)
        
        # Imprime a mensagem no console do talker
        self.get_logger().info(f'Publicando: "{msg.data}"')
        
        # Incrementa o contador para a próxima mensagem
        self.counter += 1

def main(args=None):
    # Inicializa o rclpy
    rclpy.init(args=args)

    # Cria uma instância do nó TalkerNode
    talker_node = TalkerNode()

    # Mantém o nó em execução, esperando por eventos (como o temporizador)
    # O nó será encerrado com Ctrl+C
    rclpy.spin(talker_node)

    # Destrói o nó explicitamente (opcional, mas boa prática)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
