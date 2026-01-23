pipeline {
    agent {
        dockerfile true
    }

    triggers {
        pollSCM('H * * * *')
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Run Tests') {
            steps {
                sh '''
                    mkdir -p tests/reports
                    pytest tests/ -v -s --junitxml=tests/reports/pytest-results.xml
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
