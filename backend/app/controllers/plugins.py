from app.controllers.base import Controller


CONTROLLER_REGISTRY: dict[str, type[Controller]] = {}


def register_controller(name: str, controller_class: type[Controller]) -> None:
    if not name or name in CONTROLLER_REGISTRY:
        raise ValueError(f"controller name is invalid or already registered: {name!r}")
    CONTROLLER_REGISTRY[name] = controller_class

