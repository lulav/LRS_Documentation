import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
// import styles from 'src/theme/CodeBlock/Line/styles.module.css';
export default function CodeBlockLine({
  line,
  classNames,
  showLineNumbers,
  getLineProps,
  getTokenProps,
}) {
  if (line.length === 1 && line[0].content === '\n') {
    line[0].content = '';
  }

  // returns line without '>>> ' and startsWithPrefix == true
  // or return line and startsWithPrefix == false if there were no '>>> ' at the beggining
  const [modifiedLine, startsWithPrefix] = getmodifiedLine2()

  // '>>>' may be contained in one, two or three tokens of the line (let's call them target tokens)
  // and the ' ' may be stored in the last of these tokens or in the token after the last
  function getmodifiedLine2() {
    let par = [...line]
    let isHasPref = false;
    let shift = 0
    let k = -1; //varibale to store the index of the last target token
    if (line.length > 0 && line[0].content.startsWith('>')) {
      isHasPref= true
      shift = 0
    } else {
      // the 0th token of the line may content only '' due to the condition from the beggining of the function
      // then the search of the target tokens should be shifted by 1
      if (line.length > 1 && line[0].content=='' && line[1].content.startsWith('>')) {
        isHasPref = true
        shift = 1
      }
    }
    
    if (isHasPref) {
      let i = 0 + shift; // token index
      let j = 0; // token content index
      let n = 0; //number of '>' in token

      // check first three tokens
      while ((i < line.length) && (i < (3+shift))) {
        j = 0; 
        n = 0; 

        // check the content of the each token
        while ((j < line[i].content.length) && (j < (3+shift-i))) {
          if (line[i].content[j] == '>') {
            n++; // if find '>' in the token content, record the number of '>'
          } else {
            break
          }
          j++;
        }

        // if there were '>' in token content, remove them
        if (n > 0) {
          par[i] = {...line[i], content: line[i].content.substring(n)};
          // record the token which has the last part of '>>>' to remove ' '
          k = i; //assign the index of the token to have the index of the last target token in the end
        }
        i++;
      }
    }

    // in the last token that was cleaned from the '>', check if it now has ' ' in the beggining and remove it:
    if (isHasPref && k >= 0) {
      if (par[k].content.length > 0 && par[k].content[0] == ' ') {
        par[k] = {...par[k], content: par[k].content.substring(1)};
      } else {
        // or if the ' ' in the next token, remove it from there:
        if (par.length > k && par[k+1].content.length > 0 && par[k+1].content[0] == ' ') {
          par[k+1] = {...par[k+1], content: par[k+1].content.substring(1)};
        }
      }
    }
    // return cleaned from the '>>> ' line and and true, if line contained '>>> '
    // otherwise, line itself and false
    return [par, isHasPref]
 }

  const lineProps = getLineProps({
    modifiedLine,
    className: clsx(classNames, showLineNumbers && styles.codeLine),
  });

  const lineTokens = modifiedLine.map((token, key) => (
    <span key={key} {...getTokenProps({token, key})} />
  ));

  return (
    <span {...lineProps}>
      {showLineNumbers ? (
        <>
        {startsWithPrefix ? (
          <>
          <span className={styles.unselectablePrefix}>{'>>> '}</span> 
          </>
          ) : (
          <>
          </>
          )
        }
          <span className={styles.codeLineNumber} />
          <span className={styles.codeLineContent}>{lineTokens}</span>
        </>
      ) : (
        <>
          {startsWithPrefix ? (
          <>
          <span className={styles.unselectablePrefix}>{'>>> '}</span> 
          </>
          ) : (
          <>
          </>
          )
          }
          {lineTokens}
        </>
      )}
      <br />
    </span>
  );
}
