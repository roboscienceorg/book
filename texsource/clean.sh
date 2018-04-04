sed -i -e "s/\([$]\)$/\1\n/" *.tex
sed -i s/\Open.*\[.*_.*\]// *.tex
sed -i s/$$[\ ]*$\$/$$\n$\$/ *.tex
sed -i -e "/\$\$\s*$/{
N
s/\$\$\s*\n\s*\([a-zA-Z]\)/\$\$\n\n\1/
}" *.tex

for i in $(seq 1 10 )
do

sed -i -e "/\\item/{
N
s/\n\s*\\\\/\\\\/
}" *.tex

done
