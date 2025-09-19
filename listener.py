import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    """
    Um nó ROS2 que se inscreve em um tópico e imprime as mensagens recebidas.
    """
    def __init__(self):
        super().__init__('listener')
        # Cria um assinante para o tópico 'meu_topico'
        # O tipo da mensagem (String) deve ser o mesmo do publicador
        self.subscription = self.create_subscription(
            String,
            'meu_topico',
            self.listener_callback,
            10)
        self.subscription  # previne o aviso de variável não utilizada

    def listener_callback(self, msg):
        """
        Função chamada sempre que uma nova mensagem é recebida.
        """
        # Imprime a mensagem recebida no console do listener
        self.get_logger().info(f'Recebi: "{msg.data}"')

def main(args=None):
    # Inicializa o rclpy
    rclpy.init(args=args)

    # Cria uma instância do nó ListenerNode
    listener_node = ListenerNode()

    # Mantém o nó em execução, esperando por mensagens
    # O nó será encerrado com Ctrl+C
    rclpy.spin(listener_node)

    # Destrói o nó explicitamente
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
