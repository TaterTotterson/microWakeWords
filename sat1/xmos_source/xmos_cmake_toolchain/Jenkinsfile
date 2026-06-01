@Library('xmos_jenkins_shared_library@v0.27.0') _

def localRunPytest(String extra_args="") {
    catchError{
        sh "python -m pytest --junitxml=pytest_result.xml -rA -v --durations=0 -o junit_logging=all ${extra_args}"
    }
    junit "pytest_result.xml"
}

getApproval()

pipeline {
    agent {
        label 'linux&&64'
    }

    options {
        disableConcurrentBuilds()
        skipDefaultCheckout()
        timestamps()
        buildDiscarder(xmosDiscardBuildSettings())
    }
    parameters {
        string(
            name: 'TOOLS_VERSION',
            defaultValue: '15.2.1',
            description: 'The XTC tools version'
        )
    }
    environment {
        REPO = 'xmos_cmake_toolchain'
        PYTHON_VERSION = "3.10.5"
        VENV_DIRNAME = ".venv"
    }

    stages {
        stage('Get repo') {
            steps {
                sh "mkdir ${REPO}"
                // source checks require the directory
                // name to be the same as the repo name
                dir("${REPO}") {
                    checkout scm
                    sh 'git submodule update --init --recursive --depth 1'
                }
            }
        }
        stage ("Create Python environment") {
            steps {
                dir("${REPO}") {
                    createVenv('requirements.txt')
                    withVenv {
                        sh 'pip install -r requirements.txt'
                    }
                }
            }
        }
        stage('Library checks') {
            steps {
                dir("${REPO}") {
                    sh 'git clone git@github.com:xmos/infr_apps.git'
                    sh 'git clone git@github.com:xmos/infr_scripts_py.git'
                    withVenv {
                        sh 'pip install -e infr_scripts_py'
                        sh 'pip install -e infr_apps'
                        dir("test") {
                            withEnv(["XMOS_ROOT=.."]) {
                                localRunPytest('-s test_lib_checks.py -vv')
                            }
                        }
                    }
                }
            }
        }
        stage('Tests') {
            steps {
                dir("${REPO}") {
                    withVenv {
                        withTools(params.TOOLS_VERSION) {
                            dir("test") {
                                withEnv(["XMOS_ROOT=.."]) {
                                    sh 'tox run'
                                    junit "pytest_result.xml"
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    post {
        cleanup {
            xcoreCleanSandbox()
        }
    }
}
