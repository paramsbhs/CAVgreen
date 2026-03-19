# Clean Architecture Rules

## Layer Dependency Rule

Dependencies must only point inward:

```
Infrastructure → Application → Domain
```

- **Domain** has zero ROS2 imports. Pure Python/C++ logic only.
- **Application** defines interfaces (ports). No ROS2 imports.
- **Infrastructure** implements those interfaces using ROS2.

## Entity Template (Python)

```python
# src/domain/entities/my_entity.py
from dataclasses import dataclass

@dataclass
class MyEntity:
    value: float

    def is_valid(self) -> bool:
        return self.value >= 0.0
```

## Use Case Template (Python)

```python
# src/application/use_cases/my_use_case.py
from domain.entities.my_entity import MyEntity
from application.interfaces.my_repository import MyRepository

class MyUseCase:
    def __init__(self, repository: MyRepository) -> None:
        self._repository = repository

    def execute(self, value: float) -> MyEntity:
        entity = MyEntity(value=value)
        self._repository.save(entity)
        return entity
```

## Interface (Port) Template

```python
# src/application/interfaces/my_repository.py
from abc import ABC, abstractmethod
from domain.entities.my_entity import MyEntity

class MyRepository(ABC):
    @abstractmethod
    def save(self, entity: MyEntity) -> None: ...
```
