# kitti
kitti数据集相关工具



# 仓库管理
## 五种分支

包括 master、develop、release、hotfix、feature分支
分支命名

release、hotfix 分支的命名规则分别为：release-，hotfix-。 feature分支的命名可以使用除master，develop，release-*，hotfix-*之外的任何名称。
工作顺序

1、先发issue 提交需求 2、建立对应的分支解决问题 3、完成后 push 的的commit 添加"Closes issue #1" 4、提交merge申请
工作流程

在主库已经存在的情况下，日常操作流程如下： 1、开发人员从develop主库建立分支 可以选择4种分支名字：
- feature-xxx ：新功能
- hotfix-xxx ：修复存在问题
- release-xxx ：即将发布新版本，通过release-xxx添加说明文档等辅助材料
- develop-自己的名字 ：主线开发任务推进

2、开发人员通过
 拉取gitlab的全部数据
 '''
 git pull origin 
 git pull <远程主机名> <远程分支名>
 '''

注意： git fetch origin命令，从远程获取最新版本到本地，但不会自动git merge。如果需要有选择的合并git fetch是更好的选择。效果相同时git pull将更为快捷。

    切换到你的创建的分支（这个很重要，否则你可能在别的分支进行了修改），例如
    
'''
$ git checkout . 
$ git checkout hotfix-xxx
'''
3、在文件夹中解决需求（这个是核心工作）

4、将修改好的文件上传git

'''
git add *
git commit -m "Header：fix_xxx；Body：Issue #1, #2 和 Footer：Closes #1" 
git push origin hotfix-xxx
'''

5、提交融合到develop的请求。Open a pull request

6、您的工作已经结束，感谢您的支持。 后续的工作是被有权限的开发人员进行文件融合，以及发布Tags，这个您不用担心。他们可以很好的完成这个任务。
