#~/bin/bash
echo "Start test agv mission"

source $HOME/.bashrc

for i in {1..10}
do
  echo "The $i times:"
  python mission_client.py test_final.json
#   python test.py
  sleep 10
done
