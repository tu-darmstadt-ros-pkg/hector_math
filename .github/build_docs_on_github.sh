#!/bin/bash
set -x

# file adapted from  https://tech.michaelaltfield.net/2020/07/18/sphinx-rtd-github-pages-1 by Michael Altfield <michael@michaelaltfield.net>
# added doxygen code documentation and downgraded docutils (otherwise bullet points are not shown correctly)

#####################
# GO TO DOCS FOLDER #
#####################

cd ../hector_math || { echo "Error: Failed to change directory"; exit 1; }

###################
# INSTALL DEPENDS #
###################

apt-get update
apt-get -y install git rsync python3-sphinx python3-sphinx-rtd-theme python3-stemmer python3-git python3-pip python3-virtualenv python3-setuptools

python3 -m pip install --upgrade rinohtype pygments breathe sphinxcontrib-video
python3 -m pip install -I docutils==0.16

#####################
# DECLARE VARIABLES #
#####################
export REPO_NAME="${GITHUB_REPOSITORY##*/}"
git config --global --add safe.directory /__w/${REPO_NAME}/${REPO_NAME}
pwd
ls -lah
export SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)

# make a new temp dir which will be our GitHub Pages docroot
docroot=$(mktemp -d)

##############
# BUILD DOCS #
##############

# first, cleanup any old builds' static assets
make -C docs clean
if [ $? -ne 0 ]; then
  echo "Failed to clean docs"
  exit 1
fi



# only build documentation for master branch

current_version="master"

git checkout "${current_version}"
# make the current language available to conf.py
export current_version

echo "INFO: Building sites for ${current_version}"

# skip this branch if it doesn't have our docs dir & sphinx config
if [ ! -e 'docs/conf.py' ]; then
  echo -e "\tINFO: Couldn't find 'docs/conf.py' (skipped)"
  exit 2
fi

#languages="en `find docs/locales/ -mindepth 1 -maxdepth 1 -type d -exec basename '{}' \;`"
current_language='en'

# make the current language available to conf.py
export current_language

##########
# BUILDS #
##########
echo "INFO: Building for ${current_language}"
# HTML #
sphinx-build -b html docs/ docs/_build/html/${current_language}/"${current_version}" -D language="${current_language}"

# PDF # Not working somehow rinoh fails due to @endverbatim and @see tags
#echo "Start pdf build"
#sphinx-build -b rinoh docs/ docs/_build/rinoh -D language="${current_language}"
#mkdir -p "${docroot}/${current_language}/${current_version}"
#cp "docs/_build/rinoh/target.pdf" "${docroot}/${current_language}/${current_version}/hector_math-docs_${current_language}_${current_version}.pdf"
#echo "End pdf build"

# EPUB #
sphinx-build -b epub docs/ docs/_build/epub -D language="${current_language}"
mkdir -p "${docroot}/${current_language}/${current_version}"
cp "docs/_build/epub/target.epub" "${docroot}/${current_language}/${current_version}/hector_math-docs_${current_language}_${current_version}.epub"

# copy the static assets produced by the above build into our docroot
rsync -av "docs/_build/html/" "${docroot}/"

# return to master branch
git checkout master

#######################
# Update GitHub Pages #
#######################

git config --global user.name "${GITHUB_ACTOR}"
git config --global user.email "${GITHUB_ACTOR}@users.noreply.github.com"

pushd "${docroot}" || exit 1

# don't bother maintaining history; just generate fresh
git init
git remote add deploy "https://token:${GITHUB_TOKEN}@github.com/${GITHUB_REPOSITORY}.git"
git checkout -b gh-pages

# add .nojekyll to the root so that github won't 404 on content added to dirs
# that start with an underscore (_), such as our "_content" dir..
touch .nojekyll

# add redirect from the docroot to our default docs language/version
cat >index.html <<EOF
<!DOCTYPE html>
<html>
   <head>
      <title>hector_math Docs</title>
      <meta http-equiv = "refresh" content="0; url='/${REPO_NAME}/en/master/'" />
   </head>
   <body>
      <p>Please wait while you're redirected to our <a href="/${REPO_NAME}/en/master/">master</a>.</p>
   </body>
</html>
EOF

# Add README
cat >README.md <<EOF
# GitHub Pages Cache
 
Nothing to see here. The contents of this branch are essentially a cache that's not intended to be viewed on github.com.
 
 
If you're looking to update our documentation, check the relevant development branch's 'docs/' dir.
 
For more information on how this documentation is built using Sphinx, Read the Docs, and GitHub Actions/Pages, see:
 
 * https://tech.michaelaltfield.net/2020/07/18/sphinx-rtd-github-pages-1
EOF

# copy the resulting html pages built from sphinx above to our new git repo
git add .

# commit all the new files
msg="Updating Docs for commit ${GITHUB_SHA} made on $(date -d"@${SOURCE_DATE_EPOCH}" --iso-8601=seconds) from ${GITHUB_REF} by ${GITHUB_ACTOR}"
git commit -am "${msg}"

# overwrite the contents of the gh-pages branch on our github.com repo
git push deploy gh-pages --force

popd || exit 1 # return to master repo sandbox root

# exit cleanly
exit 0
