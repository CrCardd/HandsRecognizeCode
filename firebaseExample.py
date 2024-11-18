import firebase_admin
from firebase_admin import credentials, firestore

# Caminho para o seu arquivo de chave privada JSON
cred = credentials.Certificate('caminho/para/seu/arquivo-de-chave.json')

# Inicializar o aplicativo Firebase
firebase_admin.initialize_app(cred)

# Acessar o Firestore
db = firestore.client()

# Função para adicionar dados ao Firestore
def send_data_to_firestore():
    # Referência à coleção 'users'
    users_ref = db.collection('users')
    
    # Dados que você quer adicionar ao Firestore
    data = {
        'name': 'John Doe',
        'email': 'johndoe@example.com',
        'created_at': firestore.SERVER_TIMESTAMP
    }
    
    # Adicionando o documento à coleção
    doc_ref = users_ref.add(data)
    
    print(f'Documento adicionado com ID: {doc_ref.id}')

# Chamar a função para enviar dados
send_data_to_firestore()
