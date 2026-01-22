pipeline {
    agent any

    triggers {
        pollSCM('H * * * *')
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Install dependencies') {
            steps {
                sh 'pip install -r tests/requirements.txt'
            }
        }

        stage('Run tests') {
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
