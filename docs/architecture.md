# Architecture Diagram

This document defines the high‑level architecture of the Turtle Game project.  
The diagram is expressed in Mermaid syntax, which can be rendered directly in
GitHub Markdown or converted to PNG/SVG for static documentation.

## Mermaid Diagram

```mermaid
graph TD
    %% Core components
    UI[User Interface<br/>(CLI / Web)] -->|Input| CLI[CLI Handler]
    CLI -->|Dispatch| Engine[Game Engine]
    Engine -->|Logic| Model[Model Service]
    Model -->|Generate| Response[Response Generation]
    Response -->|Output| Renderer[Renderer]
    Renderer -->|Display| User

    %% Supporting components
    Config[Config Manager] -->|Configuration| Engine
    Config -->|Settings| Model
    Test[Test Suite] -->|Validation| Engine
    Test -->|Evaluation| Model
    Telemetry[Telemetry Collector] -->|Metrics| Model
    External[Claude Code Integration] -->|Telemetry/Commands| Model

    classDef core fill:#ff9999,stroke:#333,stroke-width:2px;
    class UI,CLI,Engine,Model,Renderer,User core;
    class Config,Test,Telemetry,External fill:#99cc99,stroke:#333,stroke-width:1px;
</style>
```

## Exporting the Diagram

- **Render in GitHub**: The above Mermaid block is automatically rendered when the
  file is viewed on GitHub, providing an up‑to‑date diagram without additional
  build steps.
- **Static Image**: To generate a PNG/SVG for offline docs, install the Mermaid
  CLI (`npm install -g @mermaid-js/mermaid-cli`) and run:

  ```bash
  mmdc -i docs/architecture.md -o docs/architecture.png
  ```

  The command reads the Mermaid block and outputs a PNG image that can be
  committed alongside the markdown file.

## Usage in README

Add the following line to the README to link to this architecture diagram:

```markdown
[Architecture Overview](docs/architecture.md)
```

This provides a reproducible, version‑controlled diagram that can be updated
through normal git workflows.