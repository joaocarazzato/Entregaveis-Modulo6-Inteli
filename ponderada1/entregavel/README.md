# Turtlesim: simulando um ambiente robótico integrado no ROS

https://user-images.githubusercontent.com/99187756/233859724-c678fdcd-0e7b-4e83-bfbc-e28c5d11ab6a.mp4

## Installation

O ROS necessita da instalacao do [WSL](https://learn.microsoft.com/pt-br/windows/wsl/install) com o Ubuntu-22.04.

Após isso, é necessário instalar o [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) para seguir com nosso projeto

Entao, precisamos executar o turtlesim e o nosso script para iniciar seu funcionamento, com a comunicacao entre esses nodos:
```sh
ros2 run turtlesim turtlesim_node
```
(Em outro terminal para iniciar o script)
```sh
python3 scriptcontrol.py
```

Com isso, voce já possui o projeto com funcionamento completo.
## DIY
Com o projeto já funcionando, podemos comecar a realizar nossos próprios desenhos, mas como? Com o [rqt](https://docs.ros.org/en/humble/Concepts/About-RQt.html), instalado junto com o ROS2, podemos testar tudo isso para implementarmos no script.

Após ter testado e descobrir o que se quer fazer com o rqt, podemos iniciar o desenvolvimento através do script.

na funcao **move_turtle**, realizaremos todos os outros comandos através de outras funcoes. A partir disso, podemos utilizar as seguintes opcoes:

| Funcao | O que faz? | Parametros |
| ------ | ------ |  ------ |
| kill_turtle | Retira uma tartuga existente do campo | **name**
| color_turtle | Muda a cor do rastro de uma tartaruga | **r, g, b, width, off, name**
| pos_turtle | Muda a posicao de uma tartaruga através das coordenadas | **x, y, z, name, hz**
| spawn_turtle | Adiciona uma tartaruga ao campo | **x, y, theta(z), name**

## Guia de operacao

**Sabendo dessas funcoes, como podemos operar o projeto para desenharmos o que quisermos? Como eu fiz esse processo?**

Iremos para a funcao **move_turtle**, lá realizaremos todo o processo do zero para a criacao do desenho. Comecamos escrevendo as posicoes que gostariamos que a tartaruga se movimentasse para, fazemos isso através do rqt, para descobrirmos a posicao correta onde a tartaruga deverá ir, podemos repetir o mesmo processo para as cores do rastro, adicionar tartarugas e até mesmo tirá-las do campo, buscando criar o desenho que gostariamos de apresentar no **turtlesim**. Preenchendo os parametros da funcao, a tartaruga se movimentará de acordo com eles, terá um rastro diferente de acordo com as cores, será retirada do campo e até mesmo adicionada ao campo.

![captura_de_tela_2023-04-23_142505](https://user-images.githubusercontent.com/99187756/233859765-e4da27c6-4f1b-411a-ba60-4f9f8134b63b.png)
