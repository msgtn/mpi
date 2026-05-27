
default: deploy

setup host:
    #!/usr/bin/env bash
    set -euo pipefail
    remote_user=$(ssh {{host}} whoami)
    echo "Installing service for user: $remote_user"
    sed "s/User=dummy/User=$remote_user/" mpi.service \
        | ssh {{host}} "sudo tee /etc/systemd/system/mpi.service > /dev/null"
    ssh {{host}} "sudo systemctl daemon-reload && sudo systemctl enable mpi"
    echo "Service installed and enabled"

deploy host remote_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    rsync -av \
        --exclude 'build' \
        --exclude '.git' \
        --exclude 'mpi-mount' \
        --exclude '*.7z' \
        ./ {{host}}:{{remote_dir}}/
    ssh {{host}} "mkdir -p {{remote_dir}}/build && cd {{remote_dir}}/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF 2>&1 && make -j\$(nproc) 2>&1"
    ssh {{host}} "sudo systemctl restart mpi"
    echo "Deployed and restarted mpi service"

mount:
    sshfs dummy@192.168.90.1:/home/dummy/tapes ./mpi-mount/

sync:
    rsync -av ./mpi-mount/* ~/Pictures/mpi/ --ignore-existing
