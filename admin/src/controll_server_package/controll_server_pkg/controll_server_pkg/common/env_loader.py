from pathlib import Path
from dotenv import load_dotenv
import os

def load_env():
    # 이 파일 위치 기준으로 config/database.env 찾아감
    base_path = Path(__file__).resolve().parent.parent  # common → controll_server_pkg
    env_path = base_path / 'config' / 'database.env'
    
    if not env_path.exists():
        raise FileNotFoundError(f"❌ .env 파일을 찾을 수 없습니다: {env_path}")

    load_dotenv(dotenv_path=env_path)

    return {
        "DB_HOST": os.getenv("DB_HOST"),
        "DB_PORT": int(os.getenv("DB_PORT")),
        "DB_USER": os.getenv("DB_USER"),
        "DB_PASSWORD": os.getenv("DB_PASSWORD"),
        "DB_NAME": os.getenv("DB_NAME"),
    }
