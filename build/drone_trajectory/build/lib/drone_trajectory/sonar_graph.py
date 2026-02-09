import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range  # ou le type que ton sonar utilise
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class SonarPlotNode(Node):
    def __init__(self):
        super().__init__('sonar_plot_node')
        # Souscription au topic de ton sonar
        self.subscription = self.create_subscription(
            Range,
            '/drone/sonar',  # Remplace par ton topic
            self.listener_callback,
            10
        )
        self.data = []

        # Setup du plot
        plt.ion()  # mode interactif
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_xlim(0, 100)  # ajuster selon le nombre de points à afficher
        self.ax.set_ylim(0, 10)    # ajuster selon la plage du sonar
        self.ax.set_xlabel("Échantillon")
        self.ax.set_ylabel("Distance (m)")
        self.ax.set_title("Données sonar en temps réel")

    def listener_callback(self, msg):
        # Ajouter la donnée reçue
        self.data.append(msg.range)
        if len(self.data) > 100:  # garder seulement les 100 derniers points
            self.data.pop(0)
        self.update_plot()

    def update_plot(self):
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.set_xlim(0, max(100, len(self.data)))
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = SonarPlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
