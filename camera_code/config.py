import yaml

class FileConfig:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(FileConfig, cls).__new__(cls, *args, **kwargs)
            cls._instance._load_config()
        return cls._instance

    def _load_config(self):
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

    def get_save_path(self, data_type):
        save_paths = self.config.get('save_paths', {})
        return save_paths.get(data_type, f'/default/path/to/save/{data_type}')

# 示例使用
if __name__ == "__main__":
    config = FileConfig()
    print(f"Save path: {config.get_save_path()}")