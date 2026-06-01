@Library('xmos_jenkins_shared_library@v0.28.0') _

getApproval()

pipeline {
    agent none
    options {
        disableConcurrentBuilds()
        skipDefaultCheckout()
        timestamps()
        // on develop discard builds after a certain number else keep forever
        buildDiscarder(xmosDiscardBuildSettings())
    }

    parameters {
        string(
            name: 'TOOLS_VERSION',
            defaultValue: '15.2.1',
            description: 'The XTC tools version'
        )
        booleanParam(name: 'NIGHTLY_TEST_ONLY',
            defaultValue: false,
            description: 'Tests that only run during nightly builds.')
    }
    environment {
        REPO = 'sln_voice'
        VIEW = getViewName(REPO)
        PYTHON_VERSION = "3.8.11"
        VENV_DIRNAME = ".venv"
        BUILD_DIRNAME = "dist"
        XMOSDOC_VERSION = 'v4.0'
        VRD_TEST_RIG_TARGET = "XCORE-AI-EXPLORER"
        PIPELINE_TEST_VECTORS = "pipeline_test_vectors"
        ASR_TEST_VECTORS = "asr_test_vectors"
    }
    stages {
        stage('Build and Docs') {
            parallel {
                stage('Build and Test') {
                    when {
                        expression { !env.GH_LABEL_DOC_ONLY.toBoolean() }
                    }
                    agent {
                        label 'xcore.ai && vrd'
                    }
                    stages {
                        stage('Checkout') {
                            steps {
                                checkout scm
                                sh 'git submodule update --init --recursive --depth 1 --jobs \$(nproc)'
                            }
                        }
                        stage('Build tests') {
                            steps {
                                script {
                                    uid = sh(returnStdout: true, script: 'id -u').trim()
                                    gid = sh(returnStdout: true, script: 'id -g').trim()
                                }
                                // pull docker images
                                sh "docker pull ghcr.io/xmos/xcore_builder:latest"
                                sh "docker pull ghcr.io/xmos/xcore_voice_tester:develop"
                                // host apps
                                sh "docker run --rm -u $uid:$gid -w /sln_voice -v $WORKSPACE:/sln_voice ghcr.io/xmos/xcore_builder:latest bash -l tools/ci/build_host_apps.sh"
                                // test firmware and filesystems
                                sh "docker run --rm -u $uid:$gid -w /sln_voice -v $WORKSPACE:/sln_voice ghcr.io/xmos/xcore_builder:latest bash -l tools/ci/build_tests.sh"
                                // List built files for log
                                sh "ls -la dist_host/"
                                sh "ls -la dist/"
                            }
                        }

                        stage('ASRC Unit tests') {
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    // tools/ci/build_tests.sh does not build for x86
                                    sh "mkdir -p build_x86"
                                    sh "cmake -B build_x86 -DXCORE_VOICE_TESTS=ON"
                                    sh "cmake --build build_x86 --target test_asrc_div -j8"
                                    // x86 build
                                    sh "./build_x86/test_asrc_div"
                                    // xcore build
                                    sh "xsim dist/test_asrc_div.xe"
                                }
                            }
                        }


                        stage('ASRC Simulator') {
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    dir("test/asrc_sim") {
                                        createVenv('requirements.txt')
                                        withVenv {
                                            sh "pip install -r ./requirements.txt"
                                            sh './run.sh'
                                        }
                                    }
                                }
                            }
                        }
                        stage('Create virtual environment') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                // Create venv
                                sh "pyenv install -s $PYTHON_VERSION"
                                sh "~/.pyenv/versions/$PYTHON_VERSION/bin/python -m venv $VENV_DIRNAME"
                                // Install dependencies
                                withVenv() {
                                    sh "pip install git+https://github0.xmos.com/xmos-int/xtagctl.git"
                                    sh "pip install -r test/requirements.txt"
                                }
                            }
                        }
                        stage('Cleanup xtagctl') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                // Cleanup any xtagctl cruft from previous failed runs
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        sh "xtagctl reset_all $VRD_TEST_RIG_TARGET"
                                    }
                                }
                                sh "rm -f ~/.xtag/status.lock ~/.xtag/acquired"
                            }
                        }
                        stage('Run Sample Rate Conversion test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            withXTAG(["$VRD_TEST_RIG_TARGET"]) { adapterIDs ->
                                                sh "test/sample_rate_conversion/check_sample_rate_conversion.sh " + adapterIDs[0]
                                            }
                                            sh "pytest test/sample_rate_conversion/test_sample_rate_conversion.py --wav_file test/sample_rate_conversion/test_output/sample_rate_conversion_output.wav --wav_duration 10"
                                        }
                                    }
                                }
                            }
                        }
                        stage('Run GPIO test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            sh "test/ffd_gpio/run_tests.sh"
                                            sh 'python tools/ci/python/parse_test_output.py testing/test.rpt -outfile="testing/test_results" --print_test_results --verbose'
                                        }
                                    }
                                }
                            }
                        }
                        stage('Run FFD Low Power Audio Buffer test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            sh "test/ffd_low_power_audio_buffer/run_tests.sh"
                                            sh "pytest test/ffd_low_power_audio_buffer/test_verify_low_power_audio_buffer.py"
                                        }
                                    }
                                }
                            }
                        }
                        stage('Run Device Firmware Update test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            uid = sh(returnStdout: true, script: 'id -u').trim()
                                            gid = sh(returnStdout: true, script: 'id -g').trim()
                                            withXTAG(["$VRD_TEST_RIG_TARGET"]) { adapterIDs ->
                                                sh "docker run --rm -u $uid:$gid --privileged -v /dev/bus/usb:/dev/bus/usb -w /sln_voice -v $WORKSPACE:/sln_voice ghcr.io/xmos/xcore_voice_tester:develop bash -l test/device_firmware_update/check_dfu.sh " + adapterIDs[0]
                                            }
                                            sh "pytest test/device_firmware_update/test_dfu.py --readback_image test/device_firmware_update/test_output/readback_upgrade.bin --upgrade_image test/device_firmware_update/test_output/test_ffva_dfu_upgrade.bin"
                                        }
                                    }
                                }
                            }
                        }
                        stage('Checkout Amazon WWE') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                sh 'git clone git@github.com:xmos/amazon_wwe.git'
                            }
                        }
                        stage('Setup test vectors') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                sh "cp -r /projects/hydra_audio/xcore-voice_xvf3510_no_processing_xmos_test_suite_subset $PIPELINE_TEST_VECTORS"
                                sh "ls -la $PIPELINE_TEST_VECTORS"
                                sh "cp -r /projects/hydra_audio/xcore-voice_no_processing_ffd_test_suite $ASR_TEST_VECTORS"
                                sh "ls -la $ASR_TEST_VECTORS"
                            }
                        }
                        stage('Run FFVA Pipeline test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            withXTAG(["$VRD_TEST_RIG_TARGET"]) { adapterIDs ->
                                                sh "test/pipeline/check_pipeline.sh $BUILD_DIRNAME/test_pipeline_ffva_adec_altarch.xe $PIPELINE_TEST_VECTORS test/pipeline/ffva_quick.txt test/pipeline/ffva_test_output $WORKSPACE/amazon_wwe " + adapterIDs[0]
                                            }
                                            sh "pytest test/pipeline/test_pipeline.py --log test/pipeline/ffva_test_output/results.csv"
                                        }
                                    }
                                }
                            }
                        }
                        stage('Run FFD Pipeline test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            withXTAG(["$VRD_TEST_RIG_TARGET"]) { adapterIDs ->
                                                sh "test/pipeline/check_pipeline.sh $BUILD_DIRNAME/test_pipeline_ffd.xe $PIPELINE_TEST_VECTORS test/pipeline/ffd_quick.txt test/pipeline/ffd_test_output $WORKSPACE/amazon_wwe " + adapterIDs[0]
                                            }
                                            sh "pytest test/pipeline/test_pipeline.py --log test/pipeline/ffd_test_output/results.csv"
                                        }
                                    }
                                }
                            }
                        }
                        stage('Run ASR test') {
                            when {
                                expression { params.NIGHTLY_TEST_ONLY == true }
                            }
                            steps {
                                withTools(params.TOOLS_VERSION) {
                                    withVenv {
                                        script {
                                            withXTAG(["$VRD_TEST_RIG_TARGET"]) { adapterIDs ->
                                                sh "test/asr/check_asr.sh Sensory $ASR_TEST_VECTORS test/asr/ffd_quick.txt test/asr/sensory_output " + adapterIDs[0]
                                            }
                                            sh "pytest test/asr/test_asr.py --log test/asr/sensory_output/results.csv"
                                        }
                                    }
                                }
                            }
                        }
                    }
                    post {
                        cleanup {
                            // xcoreCleanSandbox removes all output and artifacts of the Jenkins pipeline
                            //   Comment out this post section to leave the workspace which can be useful for running items on the Jenkins agent.
                            //   However, beware that this pipeline will not run if the workspace is not manually cleaned.
                            xcoreCleanSandbox()
                        }
                    }
                }
                stage('Build docs') {
                    agent { label 'docker' }
                    stages {
                        stage ('Build docs with docker') {
                            steps {
                                checkout scm
                                sh 'git submodule update --init --recursive --depth 1 --jobs \$(nproc)'
                                sh "docker pull ghcr.io/xmos/xmosdoc:$XMOSDOC_VERSION"
                                sh """docker run -u "\$(id -u):\$(id -g)" \
                                        --rm \
                                        -v ${WORKSPACE}:/build \
                                        ghcr.io/xmos/xmosdoc:$XMOSDOC_VERSION -v"""
                                // Zip all the generated files
                                zip dir: "doc/_build/", zipFile: "xcore_voice_docs_original.zip"
                                // Rename latex folder as pdf
                                sh "mv doc/_build/latex doc/_build/pdf"
                                // Update links to latex folder in html files
                                sh "find doc/_build/html -type f -exec sed -i -e 's/latex\\/sln_voice_programming_guide_/pdf\\/sln_voice_programming_guide_/g' {} \\;"
                                sh "find doc/_build/html -type f -exec sed -i -e 's/latex\\/sln_voice_quick_start_guide_/pdf\\/sln_voice_quick_start_guide_/g' {} \\;"
                                // Remove linkcheck folder
                                sh "rm -rf doc/_build/linkcheck"
                                // Zip all the generated files
                                zip dir: "doc/_build/", zipFile: "xcore_voice_docs_release.zip"
                                // Archive doc files
                                archiveArtifacts artifacts: "xcore_voice_docs*.zip"
                            }
                        }
                    }
                    post {
                        cleanup {
                            xcoreCleanSandbox()
                        }
                    }
                }
            }
        }
    }
}
