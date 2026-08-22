from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    app_name: str = "RoboForge Studio API"
    app_version: str = "1.0.0"
    api_cors_origins: str = "http://localhost:5173,http://localhost:4173"
    simulation_timestep: float = 0.01

    model_config = SettingsConfigDict(env_file=".env", extra="ignore")

    @property
    def cors_origins(self) -> list[str]:
        return [value.strip() for value in self.api_cors_origins.split(",") if value.strip()]


@lru_cache
def get_settings() -> Settings:
    return Settings()

