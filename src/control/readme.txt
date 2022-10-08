
1.继承uav_state 需要在CMakeList.txt添加Eigen的依赖
mavros_interaction包继承uav_state





github 命令

克隆仓库：git clone <git地址>
初始化仓库：git init 

添加文件到暂存区：git add -A
把暂存区的文件提交到仓库：git commit -m "提交信息"
查看提交的历史记录：git log --stat

工作区回滚：git checkout <filename>
撤销最后一次提交：git reset HEAD^1

以当前分支为基础新建分支：git checkout -b <branchname>
列举所有的分支：git branch
单纯地切换到某个分支：git checkout <branchname>
删掉特定的分支：git branch -D <branchname>
合并分支：git merge <branchname>

推送当前分支最新的提交到远程：git push
拉取远程分支最新的提交到本地：git pull

or create a new repository on the command line
echo "# homo_geo" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/worthyland/homo_geo.git
git push -u origin main


…or push an existing repository from the command line
git remote add origin https://github.com/worthyland/homo_geo.git
git branch -M main
git push -u origin main

