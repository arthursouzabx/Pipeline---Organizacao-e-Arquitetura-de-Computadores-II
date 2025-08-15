# Pipeline RISC-V em Verilog

Este projeto implementa um processador RISC-V de 5 estágios com suporte a instruções básicas, incluindo desvios condicionais e acesso à memória, utilizando uma arquitetura pipeline.

## 📂 Estrutura do Projeto
- `src/` → Código-fonte do processador
- `docs/` → Documentação e diagramas

## 🚀 Como Rodar
1. Clone este repositório:
   ```bash
   git clone https://github.com/seu-usuario/pipeline-riscv.git
   cd pipeline-riscv

2. Com o Icarus Verilog instalado, execute no terminal do VS Code:
   ```bash
   iverilog src/pipeline-risc-v.v
   iverilog -o  top_module.vvp src/top_module.v
   vvp top_module.vvp