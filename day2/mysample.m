function Error = mysample(b)

Error = 0;
for m = 1:12
    Error = Error + (rand(1)*2) - 1;
end
Error = Error * b/6;


end