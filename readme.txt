Planejamento de movimento de robôs - Trabalho prático 1

Alunos: Giovanni, Lucca, Leandro

Instruções para execução dos scripts:
- Abra um terminal novo e execute o comando roscore
- Em um novo terminal, mude o diretório para o pacote:
cd tp1/src

1. Tangent Bug
- Em um novo terminal, execute:
rosrun stage_ros stageros worlds/map_3.world

python3 scripts/Q1_tangent_bug.py

- Digite as coordenadas (x,y) do alvo

2. Trajectory Following
- Em um novo terminal, execute:
rosrun stage_ros stageros worlds/intense_without_obstacles.world

python3 scripts/Q2_trajectory_following.py

3. Potential Field
- Em um novo terminal, execute:
rosrun stage_ros stageros worlds/map_2.world

python3 scripts/Q3_potential_field.py

- Digite as coordenadas (x,y) do alvo

4. Wavefront
- Em um novo terminal, execute:
rosrun stage_ros stageros worlds/map_1.world

python3 scripts/Q4_wavefront.py

- Digite as coordenadas (x,y) do alvo
