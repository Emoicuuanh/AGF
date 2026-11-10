# mogrify -format png $1.$2
# image2gridmap -w -i $1.png --res 0.05 -o $3.gridmap
image2gridmap -w -i $1.$2 --res $3 -o $4.gridmap --cx $5 --cy $6

YELLOW='\033[0;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
CYAN='\033[0;31m'
PURPLE='\033[0;35m'
LIGHT_GRAY='\033[0;37m'
NC='\033[0m' # No Color

echo -e "${BLUE}image2gridmap -w -i $1.$2 --res $3 -o $4.gridmap --cx $5 --cy $6${NC}"

# https://command-not-found.com/image2gridmap
# https://docs.mrpt.org/reference/latest/download-mrpt.html

# 1. sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable
# 2. sudo apt-get install mrpt-apps