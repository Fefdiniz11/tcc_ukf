# Trabalho de Conclusão de Curso de Engenharia Eletrônica

## Introdução Geral

Este projeto implementa um sistema de localização baseado no Filtro de Kalman Unscented (UKF) para um robô humanoide. O sistema visa integrar percepção, estimativa de posição e controle de movimento para navegação precisa em ambientes controlados. O robô utilizado é o NAO V6, amplamente aplicado em pesquisa e competições como a RoboCup.

## Estrutura do Código

O código é dividido em três módulos principais, uma thread de integração e a unboard:

### 1. Perception

- Detectar landmarks
- Extrair informações relevantes para alimentar o Filtro de Kalman Unscented

### 2. Localization

- Implementar o Filtro de Kalman Unscented para estimar a posição do robô.
- Atualizar continuamente a estimativa com base nas medições das landmarks e nos modelos de transição de estado.

### 3. Action

- Planejar e executar movimentos baseados nas estimativas do módulo de localização.
- Controlar a locomoção e a interação do robô com o ambiente.

### 4. Thread Principal

- Sincronizar a execução dos módulos perception, localization e action.

