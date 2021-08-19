# How to use github
THiss is a quick guide to getting started tiwht github

## Intalling git
### Download git for windows
First download Git for windows [from here](https://git-scm.com/downloads)

Then install it, all the recommended or default options are fine Git just needs
 to beon your computer. Although i recommend choosing VS Code when it asks what
 you want your default git editor to be.

### Back to VSCode
It should ahve auto installed an extension called gitlens, if it hasn't you will
 need it. Getting the official github extension is also necessary.

Once those are installed i=you should be ready to get started. if you are in VS
 code, you can bring up the command search by pressing ctrl+shift+p, enter the 
 following command:
`Git: clone`, then give it https://github.com/AdsyArena/LUNA
it should then ask you to sign in and stuff, that might have happened before this idk
 but a popup in the bottom right of your screen should be asking you if you want to
 switch to the newly cloned repository, click yes

### It should now be properly setup
and all of the stuff we are using is accessible under the source cntrol tab on the 
 left of the screen.
A couple notes:
- from the source control tab, under the remotes menu it should show that the local
 repository is connected to the github one. You may need to click the little connect
 to remote button

## Using Git
its probably best to look up some tutorials but ill give you a rundown here

### Branches
git basically works by having a bunch of branches, these are like those different
 versions of assignments you keep in case you want to go back to them.
We only have 1 branch and we will probably use more in the future but right now
 I doubt we will need more

### Getting your code onto the github
once you have done some code (typically completed a feature), you will need to commit
 it to the branch. Do this by going to the source control tab and under the first menu
 it will show you all your changes. To commit these changes if you are satisfied you
 need to add a commit message and then press commit. This will write all of the changes
 to the repository.
Notes:
- it will ask you about staging commits. Stagin a file is basically confirming you want to 
 commit it, if there are files you forgot to stage but want them commited press the yes button
- you can chose which changes you want to commit, if there are some things you dont think
 you should commit you should be able to press the "discard changes" and it wont write them
 to the github repo
- Hopefully we should not run into this, but if you get an error saying "merge conflicts" or
 anything relating to merge conflicts, worry. This is a meme among the CS community as it causes
 many headaches and is almost always a case by case fix. I've never run into it myself so im not
 sure if I will be of any help