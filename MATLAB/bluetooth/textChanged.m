function textChanged(txt, lbl, w_chr)
    write(w_chr, txt.Value);
%     set(txt,'String',''); %clear the text box after data is entered
    txt.Value = '';
end