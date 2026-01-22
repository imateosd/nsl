pipeline {
    agent {
        docker { image 'python:3.11' }
    }

    stages {
        stage('Checkout & Install') {
            steps {
                checkout scm
                sh 'pip install -r tests/requirements.txt'
            }
        }

        stage('Run Tests') {
            steps {
                sh '''
                    mkdir -p tests/reports
                    pytest tests/ --junitxml=tests/reports/pytest-results.xml
                '''
            }
        }
    }

    post {
        always {
            junit 'tests/reports/pytest-results.xml'
        }
    }
}
