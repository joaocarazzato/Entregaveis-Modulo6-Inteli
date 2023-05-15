# Gazebo: simulando um ambiente robótico integrado no ROS2 utilizando o modelo do Turtlebot 3 Burger

https://github.com/joaocarazzato/Entregaveis-Modulo6-Inteli/assets/99187756/0726ce6e-6cb2-49e1-86e8-7c8d778d5389

## Inicialização

Instale o pacote ROS utilizando o colcon e digite o seguinte comando:
```sh
ros2 run robocontrol controler
```

Com isso, voce já possui o projeto com funcionamento completo.
## DIY
Com o projeto já funcionando, podemos começar a realizar nossas próprias rotas, mas como? Com o [rqt](https://docs.ros.org/en/humble/Concepts/About-RQt.html), instalado junto com o ROS2, podemos testar tudo isso para implementarmos no script.

Após ter testado e descobrir o que se quer fazer com o rqt, podemos iniciar o desenvolvimento através do script.

Para realizarmos qualquer mudanca de posições que gostariamos de seguir, é só mudar a lista de posições da rota localizado no início do arquivo `__init__.py`.
