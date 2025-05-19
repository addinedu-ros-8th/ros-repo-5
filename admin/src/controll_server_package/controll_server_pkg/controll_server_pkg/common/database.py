import mysql.connector
from controll_server_pkg.common.env_loader import load_env

from threading import Lock

class Database:
    _instance = None
    _lock = Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:  # 멀티스레드 안전
                if cls._instance is None:
                    cls._instance = super(Database, cls).__new__(cls)
                    cls._instance._initialize()
        return cls._instance

    def _initialize(self):
        config = load_env()
        self.conn = mysql.connector.connect(
            host=config["DB_HOST"],
            port=config["DB_PORT"],
            user=config["DB_USER"],
            password=config["DB_PASSWORD"],
            database=config["DB_NAME"]
        )

    def execute_insert(self, query, params=None):
        cursor = self.conn.cursor()
        print(f"🔋 query {query} params: {params}")
        cursor.execute(query, params)
        self.conn.commit()
        last_id = cursor.lastrowid
        cursor.close()
        return last_id
    
    
    # db.update_query(
    # "UPDATE Passengers SET name = %s WHERE passenger_id = %s",
    # ("김철수", 1)
    # )


    def execute_update(self, query, params=None):
        cursor = self.conn.cursor()
        try:
            print(f"🔋 query {query} params: {params}")
            cursor.execute(query, params)
            self.conn.commit()
        finally:
            cursor.close()

    # db.delete_query(
    # "DELETE FROM Call WHERE call_id = %s",
    # (3,)
    # )

    def execute_delete(self, query, params=None):
        cursor = self.conn.cursor()
        try:
            print(f"🔋 query {query} params: {params}")
            cursor.execute(query, params)
            self.conn.commit()
        finally:
            cursor.close()

    def execute_select(self, query, params=None):
        cursor = self.conn.cursor(dictionary=True)
        
        cursor.execute(query, params)
        print(f"🔋 query {query} params: {params}")
        result = cursor.fetchall()
        cursor.close()
        return result

    def close(self):
        self.conn.close()
        Database._instance = None  # 명시적으로 인스턴스 제거 가능
