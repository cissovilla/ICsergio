Lá temos três pastas:
PCL com todos os meus teste em PCL.
Para rodar basta ir na pasta build e chamar o cmake
# cmake ..

Depois chamar o make

# make

Pronto agora para ver meu teste de bounding box
basta abrir:
# ./pcl_bounding_box_demo
este teste tem dois parâmetros o primeiro é o arquivo de entrada .ply e o segundo a quantidade de graus que se quer aplicar o twisting.
um exemplo bom é esse:
#./pcl_bounding_box_demo ./Armadillo.ply 180

para passar cada tempo (eu dividi em 20 tempos), basta apertar a tecla 'q'.


Em CUDA temos os testes de cuda nada demais. mas agora já funcionando. antes não estava pegando.

Em PCL-CUDA tem meus primeiros teste de iteração do pcl e do cuda mas nenhum resultado positivo ainda não consegui fazer os dois rodarem junto. estou pesquisando o cmake , mas esta difícil.
