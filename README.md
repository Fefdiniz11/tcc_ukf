# Trabalho de Conclusão de Curso de Engenharia Eletrônica

Este projeto implementa um sistema de localização baseado no Filtro de Kalman Unscented (UKF) para um robô humanoide. O sistema visa integrar percepção, estimativa de posição e controle de movimento para navegação precisa em ambientes controlados. O robô utilizado é o NAO V6, amplamente aplicado em pesquisa e competições como a RoboCup.

## Tutorial: Rodando o UKF no Robô NAO

### Requisitos
Para rodar os códigos de localização com Filtro de Kalman Unscented (UKF) no robô NAO, é necessário que a versão 2.8 da NAOqi esteja instalada no robô. Além disso, as seguintes dependências devem ser instaladas:

```bash
pip install scipy --user
pip install numpy --user
pip install --user filterpy --no-deps
```

> **Nota:** O matplotlib não é instalado, pois o robô NAO não possui interface gráfica.

---

### Transferindo o Código para o Robô
Os códigos estão organizados em pastas:
- `ukf-angle` (Filtro de Kalman Unscented para orientação)
- `ukf-localization` (Filtro de Kalman Unscented para localização)

Para transferir qualquer uma dessas pastas para o robô, compacte a pasta desejada em um arquivo `.zip` e utilize o comando `scp` para enviá-la ao robô.

### Exemplo de envio para o robô:
```bash
zip -r ukf-localization.zip ukf-localization
scp ukf-localization.zip nao@<IP_DO_ROBO>:/home/nao/
```

> **Nota:** Substitua `<IP_DO_ROBO>` pelo IP real do robô NAO na rede.

Depois de transferir o arquivo, acesse o robô via SSH e descompacte o projeto:
```bash
ssh nao@<IP_DO_ROBO>
cd /home/nao/
unzip ukf-localization.zip
```

---

### Executando o UKF no Robô NAO
Depois de acessar o robô e entrar na pasta do projeto, execute o seguinte comando:
```bash
python threads.py
```
Isso iniciará a execução do UKF no robô.

> **Dica:** Certifique-se de que as dependências estão corretamente instaladas antes de rodar o código.

---

### Uso de Landmarks
Para o funcionamento adequado da localização com UKF, é necessário o uso de landmarks. Neste projeto, foram utilizadas as landmarks 64 e 108, mas esses valores podem ser alterados conforme a necessidade.

Com isso, o código do UKF estará rodando no robô NAO com os dados de sensores e movimentação integrados.

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

